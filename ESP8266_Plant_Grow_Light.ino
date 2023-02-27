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
 * 
 * Hardware Theory of Operation:
 * ---
 * Soldered onto PCB ~
 * 
 *  Blue Wire = Pad S2 = Dimming Button (BTN 2)
 *      - Level shifter to ESP for input
 *      - ESP to MOFET for output
 * 
 *  White Wire = Pad S3 = Power Button (BTN 1)
 *      - Level shifter to ESP for input
 *      - ESP to MOFET for output
 * 
 *  Green Wire = Pad S4 = Timing Button (BTN 3)
 *      - Level shifter to ESP for input
 *      - ESP to MOFET for output
 * 
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
#define __NUM_DIMMER_STATES__                   (5)
#define __NUM_TIMER_STATES__                    (4)

//==============================================================================
//  Preprocessor Macros
//==============================================================================

#define readResetBtnState()     ((bool)digitalRead(RESET_BTN_PIN))
#define readPowerPin()          ((bool)digitalRead(READ_PWR_PIN))
#define readDimmerPin()         ((bool)digitalRead(READ_DIM_PIN))
#define readTimerPin()          ((bool)digitalRead(READ_TIM_PIN))

#define writeStatusLED(state)   (digitalWrite(STATUS_LED_PIN, (bool)(state)))

#define _pulseBtn(pin) {                                                       \
    Serial.println("Pulse " #pin);                                             \
    digitalWrite(pin, HIGH);                                                   \
    delay(PULSE_TIME);                                                         \
    digitalWrite(pin, LOW);                                                    \
    delay(1);                                                                  \
}

#define pulsePowerBtn()         (_pulseBtn(CTRL_PWR_PIN))
#define pulseDimmerBtn()        (_pulseBtn(CTRL_DIM_PIN))
#define pulseTimerBtn()         (_pulseBtn(CTRL_TIM_PIN))

//==============================================================================
//  Enumerated Constants
//==============================================================================

typedef enum _led_channel {
    LED_OFF,
    LED_ALL,
    LED_CHA,
    LED_CHB
} led_channel_t;

typedef enum _led_dimmer {
    LED_DIM_0,
    LED_DIM_1,
    LED_DIM_2,
    LED_DIM_3,
    LED_DIM_4
} led_dimmer_t;

typedef enum _led_timer {
    LED_TIMER_OFF,
    LED_TIMER_3HR,
    LED_TIMER_6HR,
    LED_TIMER_12HR
} led_timer_t;

//==============================================================================
//  Data Structure Declaration
//==============================================================================

typedef struct _led_state {
    led_channel_t ch;
    led_dimmer_t dim;
    led_timer_t tim;
} led_state_t;

typedef struct _soft_timer {
    String timestamp;
    int hours;
    unsigned long _targetTime; // use getEpochTime()
} soft_timer_t;

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
static bool _hasActiveRPC = false;
static bool _softTimerActive = false;
static char _activeRPCUUID[50];
static led_state_t _currentState, _targetState;
static soft_timer_t _softTimer;

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static unsigned long _blinkDurationByState();
static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc);
static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc);
static void _pollRPCResponse(polip_device_t* dev, JsonDocument& doc);
static void _pushRPCSetup(polip_device_t* dev, JsonDocument& doc);
static void _pushRPCResponse(polip_device_t* dev, JsonDocument& doc);
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
    _polipWorkflow.params.pollRPC = false;
    _polipWorkflow.hooks.pushStateSetupCb = _pushStateSetup;
    _polipWorkflow.hooks.pollStateRespCb = _pollStateResponse;
    _polipWorkflow.hooks.pollRPCRespCb = _pollRPCResponse;
    _polipWorkflow.hooks.workflowErrorCb = _errorHandler;
    _polipWorkflow.hooks.pushRPCSetupCb = _pushRPCSetup;
    _polipWorkflow.hooks.pushRPCRespCb = _pushRPCResponse;

    unsigned long currentTime = millis();
    polip_workflow_initialize(&_polipWorkflow, currentTime);
    _cooldownTime = _blinkTime = _resetTime = currentTime;
    _prevBtnState = readResetBtnState();
    _flag_reset = false;

    _currentState.ch = LED_OFF;
    _currentState.dim = LED_DIM_4;
    _currentState.tim = LED_TIMER_OFF;

    _targetState.ch = LED_OFF;
    _targetState.dim = LED_DIM_4;
    _targetState.tim = LED_TIMER_OFF;
}

void loop() {
    // We do most things against soft timers in the main loop
    unsigned long currentTime = millis();

    // Refresh time
    _timeClient.update();

    // Monitor soft timer
     if (_softTimerActive && _timeClient.getEpochTime() >= _softTimer._targetTime) {
        _targetState.ch = LED_OFF;
        _softTimerActive = false;
        POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
    }

    // Update Polip Server
    polip_workflow_periodic_update(&_polipWorkflow, _doc, _timeClient.getFormattedDate().c_str(), currentTime);

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
                Serial.println(_currentState.ch);
            } else if (str == "dim?") { 
                Serial.println(_currentState.dim);
            } else if (str == "timer?") { 
                Serial.println(_targetState.tim);
                Serial.println(_softTimerActive);
                Serial.println(_softTimer.timestamp);
                Serial.println(_softTimer.hours);
            } else if (str == "pon") { 
                _targetState.ch = LED_ALL;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "poff") { 
                _targetState.ch = LED_OFF;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "pcha") { 
                _targetState.ch = LED_CHA;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "pchb") { 
                _targetState.ch = LED_CHB;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "dim0") { 
                _targetState.dim = LED_DIM_0;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "dim1") { 
                _targetState.dim = LED_DIM_1;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "dim2") { 
                _targetState.dim = LED_DIM_2;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "dim3") { 
                _targetState.dim = LED_DIM_3;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "dim4") { 
                _targetState.dim = LED_DIM_4;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
            } else if (str == "toff") { 
                _targetState.tim = LED_TIMER_OFF;
                POLIP_WORKFLOW_STATE_CHANGED(&_polipWorkflow);
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
    if ((currentTime - _cooldownTime) > COOLDOWN_THRESHOLD) {
        _cooldownTime = currentTime;

        if (_currentState.ch != _targetState.ch) {
            _currentState.ch = (led_channel_t)(((int)_currentState.ch + 1) % __NUM_POWER_STATES__);
            pulsePowerBtn();
        } else if (_currentState.tim != _targetState.tim && _currentState.ch != LED_OFF) {
            _currentState.tim = (led_timer_t)(((int)_currentState.tim + 1) % __NUM_TIMER_STATES__);
            pulseTimerBtn();
        } else if (_currentState.dim != _targetState.dim && _currentState.ch != LED_OFF) {
            _currentState.dim = (led_dimmer_t)(((int)_currentState.dim + 1) % __NUM_DIMMER_STATES__);
            pulseDimmerBtn();
        }
    }

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

static unsigned long _blinkDurationByState() {
    if (_currentState.ch == LED_OFF) {
        return 2000L; // Long pulse = off
    } else if (_currentState.tim != LED_TIMER_OFF) {
        return 100L; // Very fast = error; timer should not be set
    } else {
        return 500L + 100L * _currentState.dim;
    }
}

static void _pushStateSetup(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc.createNestedObject("state");
    stateObj["power"] = (_targetState.ch == LED_OFF) ? "off" : "on";
    stateObj["intensity"] = _targetState.dim;

    if (_targetState.ch == LED_OFF || _targetState.ch == LED_ALL) {
        stateObj["channels"] = "all";
    } else if (_targetState.ch == LED_CHA) {
        stateObj["channels"] = "a";
    } else if (_targetState.ch == LED_CHB) {
        stateObj["channels"] = "b";
    }
    
    if (_softTimerActive) {
        JsonObject timer = stateObj.createNestedObject("state");
        timer["timestamp"] = _softTimer.timestamp;
        timer["duration"] = _softTimer.hours;
    } else {
        stateObj["timer"] = nullptr;
    }
}

static void _pollStateResponse(polip_device_t* dev, JsonDocument& doc) {
    JsonObject stateObj = doc["state"];
    String power = stateObj["power"];
    int intensity = stateObj["intensity"];
    String channels = stateObj["channels"];
    
    if (power == "off") {
        _targetState.ch = LED_OFF;
    } else if (power == "on") {
        if (channels == "all") {
            _targetState.ch = LED_ALL;
        } else if (channels == "a") {
            _targetState.ch = LED_CHA;
        } else if (channels == "b") {
            _targetState.ch = LED_CHB;
        }

        if (intensity >= 0 && intensity < __NUM_DIMMER_STATES__) {
            _targetState.dim = (led_dimmer_t)intensity;
        }
    }
    
    // Note ignore timer (must be set via RPC)
}

static void _pollRPCResponse(polip_device_t* dev, JsonDocument& doc) {
    if (!_hasActiveRPC) { // Only accept RPC if not already working on one
        JsonObject rpcObj = doc["rpc"];
        String uuid = rpcObj["uuid"]; 
        String type = rpcObj["type"];

        if (type == "timer") {
            //parameters parse
            bool paramsParsedSuccessfully = false;
            JsonArray array = rpcObj["parameters"].as<JsonArray>();
            for(JsonObject param : array) {
                if (param.containsKey("duration")) {
                    _softTimer.timestamp = _timeClient.getFormattedDate();
                    _softTimer.hours = param["duration"];
                    _softTimer._targetTime = 60 * 60 * _softTimer.hours + _timeClient.getEpochTime();
                    paramsParsedSuccessfully = true;
                }
            }

            if (paramsParsedSuccessfully) {
                _hasActiveRPC = true;
                strcpy(_activeRPCUUID, uuid.c_str());
                POLIP_WORKFLOW_RPC_FINISHED(&_polipWorkflow);
            } else {
                Serial.println("Failed to parse soft timer RPC");
            }
        }
    }
}

static void _pushRPCSetup(polip_device_t* dev, JsonDocument& doc) {
    JsonObject rpcObj = doc.createNestedObject("rpc");
    rpcObj["uuid"] = _activeRPCUUID;
    rpcObj["result"] = "OK";
    
}

static void _pushRPCResponse(polip_device_t* dev, JsonDocument& doc) {
    _hasActiveRPC = false;
}

static void _errorHandler(polip_device_t* dev, JsonDocument& doc, polip_workflow_source_t source) { 
    Serial.print(F("Error Handler ~ polip server error during OP="));
    Serial.print((int)source);
    Serial.print(F(" with CODE="));
    Serial.println((int)_polipWorkflow.flags.error);
} 
