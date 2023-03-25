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
 *  [home] -> Request rehoming LEDcontroller to known state
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
 *  Yellow Wires (2) = Channel PWM lines = connected through comparator to MUX
 * 
 *  Yellow Wires (2) = Timer LED lines = connected through comparator to MUX
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
 *   A0 MUX input summary state of LED controller
 *   D5 Control power button
 *   D6 Control dimmer button
 *   D7 Control timer button
 *   D0 Read power button
 *   D1 Read dimmer button
 *   D2 Read timer button
 *   D3 for wifi-manager reset button (pulled high for our button)
 *   D8 MUX control for summary input state state
 * 
 * Analog Input MUX:
 *  D8 = LOW -> A0 is PWMing or Fully On
 *  D8 = High -> A0 is Timer is not manual (bad state)
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

//==============================================================================
//  Preprocessor Constants
//==============================================================================

#define CTRL_PWR_PIN                            (D0)
#define CTRL_DIM_PIN                            (D7)
#define CTRL_TIM_PIN                            (D6)
#define READ_PWR_PIN                            (D5)
#define READ_DIM_PIN                            (D1)
#define READ_TIM_PIN                            (D2)
#define RESET_BTN_PIN                           (D3)
#define STATUS_LED_PIN                          (LED_BUILTIN)
#define SUMMARY_MUX_CTRL_PIN                    (D8)
#define SUMMARY_MUX_AIN_PIN                     (A0)

#define RESET_BTN_TIME_THRESHOLD                (200L)      //ms
#define PULSE_TIME                              (100L)      //ms
#define COOLDOWN_THRESHOLD                      (500L)      //ms
#define MUX_SETTLE_TIME                         (1000L)     //us
#define SETUP_HW_STABLE_DELAY                   (100L)      //ms

#define DEBUG_SERIAL_BAUD                       (115200)    //baud

#define ANALOG_ON_THRESHOLD                     (512) // ADC is 2^10

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

#define _pulseBtn(pin, flag) {                                                  \
    Serial.println("Pulse " #pin);                                              \
    flag = true;                                                                \
    digitalWrite(pin, HIGH);                                                    \
    delay(PULSE_TIME);                                                          \
    digitalWrite(pin, LOW);                                                     \
    delay(1);                                                                   \
}

#define pulsePowerBtn()         (_pulseBtn(CTRL_PWR_PIN, _activePulse_power))
#define pulseDimmerBtn()        (_pulseBtn(CTRL_DIM_PIN, _activePulse_dimmer))
#define pulseTimerBtn()         (_pulseBtn(CTRL_TIM_PIN, _activePulse_timer))

#define warmUpADC() {for (int i=0; i<2; i++) {                                  \
        analogRead(SUMMARY_MUX_AIN_PIN);                                        \
}}

static bool readStateSummary_ledsOn(void) {
    digitalWrite(SUMMARY_MUX_CTRL_PIN, LOW);
    delayMicroseconds(MUX_SETTLE_TIME);
    return analogRead(SUMMARY_MUX_AIN_PIN) > ANALOG_ON_THRESHOLD;
}

static bool readStateSummary_timerOn(void) {
    digitalWrite(SUMMARY_MUX_CTRL_PIN, HIGH);
    delayMicroseconds(MUX_SETTLE_TIME);
    return analogRead(SUMMARY_MUX_AIN_PIN) > ANALOG_ON_THRESHOLD;
}

//==============================================================================
//  Private Module Variables
//==============================================================================

static bool _activePulse_power, _activePulse_dimmer, _activePulse_timer;

//==============================================================================
//  MAIN
//==============================================================================

void setup(void) {
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.flush();

    Serial.println("Basic Hardware Setup");

    pinMode(CTRL_PWR_PIN, OUTPUT);
    pinMode(CTRL_DIM_PIN, OUTPUT);
    pinMode(CTRL_TIM_PIN, OUTPUT);
    pinMode(READ_PWR_PIN, INPUT);
    pinMode(READ_DIM_PIN, INPUT);
    pinMode(READ_TIM_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(RESET_BTN_PIN, INPUT); 
    pinMode(SUMMARY_MUX_CTRL_PIN, OUTPUT);

    digitalWrite(CTRL_PWR_PIN, LOW);
    digitalWrite(CTRL_DIM_PIN, LOW);
    digitalWrite(CTRL_TIM_PIN, LOW);
    digitalWrite(SUMMARY_MUX_CTRL_PIN, LOW);

    warmUpADC();

    delay(SETUP_HW_STABLE_DELAY);
    Serial.println("Ready");
}

void loop(void) {

    //pulsePowerBtn();
    //pulseDimmerBtn();
    //pulseTimerBtn();

    //bool status = readStateSummary_ledsOn();
    //bool status = readStateSummary_timerOn();
    bool status = readResetBtnState();
    Serial.println(status);

    delay(500);
}