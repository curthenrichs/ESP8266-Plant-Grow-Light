/**
 * @file ESP8266_Plant_Grow_Light.ino
 * @author Curt Henrichs
 * @brief ESP8266 Plant Frow Light
 * @version 0.1
 * @date 2023-02-22
 * @copyright Copyright (c) 2023
 * 
 * Controls "hacked" plant grow light using ESP8266. 
 * 
 * This device uses standard Polip-lib workflow to push current state on
 * unexpected change, poll server for its current state of this device, and
 * interface with the get value behavior for server-client communication. We
 * will not use push sensors or push errors in this code.
 * 
 * Dependency installation:
 * ---
 * - Arduino_JSON (via Arduino IDE)
 * - ESP8266 Board (via Arduino IDE)
 * - WiFiManager (via Arduino IDE)
 * - NTPClient (via Arduino IDE)
 * - Arduino Crypto (via Arduino IDE)
 *      - (I had to manually rename the library from Crypto.h to ArduinoCrypto.h)
 * 
 * Debug Serial:
 * ---
 * Several commands available for testing behavior
 * 
 *  [reset] -> Requests hardware state reset (will wipe EEPROM)
 *  [power?] -> Gets current power state
 *  [dim?] -> Gets current dimmer state
 *  [timer?] -> Gets current timer state
 *  [error?] -> Queries error status of polip lib
 *  [pon] -> Sets power to full on (cha, chb)
 *  [poff] -> Sets power to off
 *  [pcha] -> Sets power to cha on, chb off
 *  [pchb] -> Sets power to cha off, chb on
 *  [dim0] -> Sets dim level 0
 *  [dim1] -> Sets dim level 1
 *  {dim2} -> Sets dim level 2
 *  {dim3} -> Sets dim level 3
 *  [toff] -> Sets timer to off
 *  [t3hr] -> Sets timer to 3 hrs
 *  [t6hr] -> Sets timer to 6 hrs
 *  [t12hr] -> Sets timer to 12 hrs
 * 
 * Hardware Theory of Operation:
 * ---
 * Soldered onto PCB ~
 *  Blue Wire = Pad S2 = Dimming Button (BTN 2)
 *      - Level shifter to ESP for input
 *      - ESP to MOFET for output
 *  White Wire = Pad S3 = Power Button (BTN 1)
 *      - Level shifter to ESP for input
 *      - ESP to MOFET for output
 *  Green Wire = Pad S4 = Timing Button (BTN 3)
 *      - Level shifter to ESP for input
 *      - ESP to MOFET for output
 *  Yellow Wires (2) = Channel PWM lines = Not Connected
 * 
 * Timer displays state via red and blue LEDs
 *  OFF = Manual
 *  RED = 3
 *  BLUE = 6
 *  RED and BLUE = 12
 * 
 * For Buttons ~
 *  We tied each button to an input line for unexpected state change tracking
 *  and a MOSFET actuated by ESP8266
 *  Fast click to ground jogs through button state
 *  
 * For ESP8266 Pinout, followed guide ~
 * https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
 *   Builtin LED for status
 *   D5 Control power button
 *   D6 Control dimmer button
 *   D7 Control timer button
 *   D0 Read power button
 *   D1 Read dimmer button
 *   D2 Read timer button
 *   D3 for wifi-manager reset button (pulled high for our button)
 * 
 * Tracking state transition:
 * ---
 * All queued button presses are tracked as an integer counter. When a new press
 * is triggered, the firmware flips a bool to true. State change tracker
 * checks this flag to determine if the state change was induced by firmware
 * or user press. If unexpected change (transition bool is false but state
 * changed) then we know to push new state to polip server.
 * 
 * States:
 * ---
 *  Power ~
 *   (Start) -> (Off) -> (all) -> (cha on, chb off) -> (cha 0ff, chb on) -
 *                ^                                                      |
 *                |                                                      |
 *                --------------------------------------------------------
 * 
 *  (When Timer expires) -> (currentState) -> (off)
 * 
 * Dimmer ~
 *  (When Power Off -> all) -> (Dim 3 "Full") -> (Dim 0) -> (Dim 1) -> (Dim 2) -
 *                                   ^                                         |
 *                                   |                                         |
 *                                   -------------------------------------------
 * 
 *  (When Power != off -> off) -> DON'T CARE
 * 
 * Timer ~
 *  (When Power Off -> all) -> (Off) -> (3hr) -> (6hr) -> (12hr) -
 *                               ^                               |
 *                               |                               |
 *                               ---------------------------------
 * 
 *  (When Power != off -> off) -> DON'T CARE
 */

//==============================================================================
//  Libraries
//==============================================================================

#include <Arduino.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <polip-client.h>
#include <ESP8266HTTPClient.h>

//==============================================================================
//  Preprocessor Constants
//==============================================================================

#define CTRL_PWR_PIN                            (D5)
#define CTRL_DIM_PIN                            (D6)
#define CTRL_TIM_PIN                            (D7)
#define READ_PWR_PIN                            (D0)
#define READ_DIM_PIN                            (D1)
#define READ_TIM_PIN                            (D2)
#define RESET_BTN_PIN                           (D3)
#define STATUS_LED_PIN                          (LED_BUILTIN)

#define RESET_BTN_TIME_THRESHOLD                (200L)
#define PULSE_TIME                              (100L)
#define COOLDOWN_THRESHOLD                      (500L)

#define DEBUG_SERIAL_BAUD                       (115200)

#define NTP_URL                                 "pool.ntp.org"
#define FALLBACK_AP_NAME                        "AP-ESP8266-Plant-Grow-Light"

#define __NUM_POWER_STATES__                    (4)
#define __NUM_DIMMER_STATES__                   (4)
#define __NUM_TIMER_STATES__                    (4)

//==============================================================================
//  Preprocessor Macros
//==============================================================================

#define readResetBtnState()     ((bool)digitalRead(RESET_BTN_PIN))
#define readPowerPin()          ((bool)digitalRead(READ_PWR_PIN))
#define readDimmerPin()         ((bool)digitalRead(READ_DIM_PIN))
#define readTimerPin()          ((bool)digitalRead(READ_TIM_PIN))

#define writeStatusLED(state)   (digitalWrite(STATUS_LED_PIN, (bool)(state)))

#define _pulseBtn(pin) {
    Serial.println("Pulse " #pin);
    digitalWrite(pin, HIGH);
    delay(PULSE_TIME);
    digitalWrite(pin, LOW);
}

#define pulsePowerBtn()         (_pulseBtn(CTRL_PWR_PIN))
#define pulseDimmerBtn()        (_pulseBtn(CTRL_DIM_PIN))
#define pulseTimerBtn()         (_pulseBtn(CTRL_TIM_PIN))

//==============================================================================
//  Enumerated Constants
//==============================================================================

//==============================================================================
//  Data Structure Declaration
//==============================================================================

//==============================================================================
//  Declared Constants
//==============================================================================

const char* SERIAL_STR = "grow-light-0-0000";
const char* KEY_STR = "revocable-key-0";    //NOTE: Should be configurable
const char* HARDWARE_STR = POLIP_VERSION_STD_FORMAT(0,0,1); 
const char* FIRMWARE_STR = POLIP_VERSION_STD_FORMAT(0,0,1);

//==============================================================================
//  Private Module Variables
//==============================================================================

static StaticJsonDocument<POLIP_MIN_RECOMMENDED_DOC_SIZE> _doc;
static WiFiUDP _ntpUDP;
static NTPClient _timeClient(_ntpUDP, NTP_URL, 0);
static WiFiManager _wifiManager;
static polip_device_t _polipDevice;
static polip_workflow_t _polipWorkflow;

static bool _blinkState = false;
static unsigned long _blinkTime, _resetTime, _cooldownTime;
static char _rxBuffer[50];
static int _rxBufferIdx = 0;
static bool _prevBtnState = false;
static bool _flag_reset = false;

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static unsigned long _blinkDurationByState(/*TODO*/);
static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc);
static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc);
static void _errorHandler(polip_device_t* dev, JsonDocument& doc, polip_workflow_source_t source);

//==============================================================================
//  MAIN
//==============================================================================

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.flush();

    pinMode(CTRL_PWR_PIN, OUTPUT);
    pinMode(CTRL_DIM_PIN, OUTPUT);
    pinMode(CTRL_TIM_PIN, OUTPUT);
    pinMode(READ_PWR_PIN, INPUT);
    pinMode(READ_DIM_PIN, INPUT);
    pinMode(READ_TIM_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(RESET_BTN_PIN, INPUT); 

    digitalWrite(CTRL_PWR_PIN, LOW);
    digitalWrite(CTRL_DIM_PIN, LOW);
    digitalWrite(CTRL_TIM_PIN, LOW);

    _wifiManager.autoConnect(FALLBACK_AP_NAME);

    _timeClient.begin();

    POLIP_BLOCK_AWAIT_SERVER_OK();

    _polipDevice.serialStr = SERIAL_STR;
    _polipDevice.keyStr = (const uint8_t*)KEY_STR;
    _polipDevice.keyStrLen = strlen(KEY_STR);
    _polipDevice.hardwareStr = HARDWARE_STR;
    _polipDevice.firmwareStr = FIRMWARE_STR;
    _polipDevice.skipTagCheck = false;

    _polipWorkflow.device = &_polipDevice;
    _polipWorkflow.hooks.pushStateSetupCb = _pushStateSetup;
    _polipWorkflow.hooks.pollStateRespCb = _pollStateResponse;
    _polipWorkflow.hooks.workflowErrorCb = _errorHandler;

    unsigned long currentTime = millis();
    polip_workflow_initialize(&_polipWorkflow, currentTime);
    _cooldownTime = _blinkTime = _resetTime = currentTime;
    _prevBtnState = readResetBtnState();
    _flag_reset = false;
}

void loop() {
    // We do most things against soft timers in the main loop
    unsigned long currentTime = millis();

    // Refresh time
    _timeClient.update();

    // Acknowledge button press state changes
    //TODO

    // Update Polip Server
    polip_workflow_periodic_update(&_polipWorkflow, _doc, _timeClient.getFormattedTime().c_str(), currentTime);

    // Serial debugging interface provides full state control
    while (Serial.available() > 0) {
        if (_rxBufferIdx >= (sizeof(_rxBuffer) - 1)) {
            Serial.println(F("Error - Buffer Overflow ~ Clearing"));
            _rxBufferIdx = 0;
        }

        char c = Serial.read();
        if (c != '\n') {
            _rxBuffer[_rxBufferIdx] = c;
            _rxBufferIdx++;
        } else {
            _rxBuffer[_rxBufferIdx] = '\0';
            _rxBufferIdx = 0;
      
            String str = String(_rxBuffer);
      
            if (str == "reset") {
                Serial.println(F("Debug Reset Requested"));
                _flag_reset = true;
            } else if (str == "power?") { 
                
            } else if (str == "dim?") { 
                
            } else if (str == "timer?") { 
                
            } else if (str == "pon") { 
                
            } else if (str == "poff") { 
                
            } else if (str == "pcha") { 
                
            } else if (str == "pchb") { 
                
            } else if (str == "dim0") { 
                
            } else if (str == "dim1") { 
                
            } else if (str == "dim2") { 
                
            } else if (str == "dim3") { 
                
            } else if (str == "toff") { 
                
            } else if (str == "t3hr") { 
                
            } else if (str == "t6hr") { 
                
            } else if (str == "t12hr") { 
                
            } else if (str == "error?") {
                if (POLIP_WORKFLOW_IN_ERROR(&_polipWorkflow)) {
                    Serial.println(F("Error in PolipLib: "));
                    Serial.println((int)_polipWorkflow.flags.error);
                } else {
                    Serial.println(F("No Error"));
                }
                POLIP_WORKFLOW_ACK_ERROR(&_polipWorkflow);
            } else {
                Serial.print(F("Error - Invalid Command ~ `"));
                Serial.print(str);
                Serial.println(F("`"));
            }
        }
    }

    // Handle Reset button
    bool btnState = readResetBtnState();
    if (!btnState && _prevBtnState) {
        _resetTime = currentTime;
    } else if (btnState && !_prevBtnState) {
        if ((currentTime - _resetTime) >= RESET_BTN_TIME_THRESHOLD) {
            Serial.println("Reset Button Held");
            _flag_reset = true;
        }
        // Otherwise button press was debounced.
    }
    _prevBtnState = btnState;

     // Reset State Machine
    if (_flag_reset) {
        _flag_reset = false;
        Serial.println("Erasing Config, restarting");
        _wifiManager.resetSettings();
        ESP.restart();
    }

    // Handle our queued pulses. This will block for one pulse at a time!
    //TODO

    // Blink the LED at a certain rate dependent on state of purifier
    //  Useful for debugging
    if ((currentTime - _blinkTime) > _blinkDurationByState()) {
        _blinkTime = currentTime;
        writeStatusLED(_blinkState);
        _blinkState = !_blinkState;
    }

    delay(1);
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

static unsigned long _blinkDurationByState(/*TODO*/) {
    return 0; //TODO
}

static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc.createNestedObject("state");
    stateObj["power"] = "off";
    stateObj["intensity"] = 0;
    stateObj["channels"] = "all";

    JsonObject timer = stateObj.createNestedObject("state");
    timer["duration"] = "off";
    timer["timestamp"] = "";
    //TODO
}

static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc["state"];
    const char* power = stateObj["power"];
    int intensity = stateObj["intensity"];
    const char* channels = stateObj["channels"];
    const char* timer = stateObj["timer"];
    //TODO
}

static void _errorHandler(polip_device_t* dev, JsonDocument& doc, polip_workflow_source_t source) { 
    Serial.print(F("Error Handler ~ polip server error during OP="));
    Serial.print((int)source);
    Serial.print(F(" with CODE="));
    Serial.println((int)_polipWorkflow.flags.error);
} 