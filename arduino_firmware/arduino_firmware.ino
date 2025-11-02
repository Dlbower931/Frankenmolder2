/**
 * Frankenmolder - ESP32 Main Control Unit (v2 - True PID)
 *
 * Implements full PID control with PWM and a CAN watchdog.
 * 1. Reads 3x MAX6675 sensors.
 * 2. Receives Setpoint (float) and State (2-byte) Commands from Pi.
 * 3. Runs State Machine (OFF, HEATING, PID) for each zone.
 * 4. Runs a true PID controller for each zone, outputting PWM (0-255).
 * 5. Controls 3x Heater GPIO pins via PWM.
 * 6. Publishes actual Temperature (float) and actual State (String) back to Pi.
 *
 * REQUIRED LIBRARIES:
 * 1. ESP32-TWAI-CAN (by rdupuis)
 * 2. Adafruit MAX6675 Library (by Adafruit)
 * 3. PID (by Brett Beauregard)
 */

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <max6675.h> // Adafruit's library
#include <PID_v1.h>  // The PID library

// --- Pin Definitions ---
// CAN
#define CAN_TX 5 // (TXD)
#define CAN_RX 4 // (RXD)
// MAX6675 Software SPI
#define SCK_PIN 18
#define SO_PIN  19
#define CS1_PIN 13
#define CS2_PIN 14
#define CS3_PIN 27
// Heater Control (PWM)
#define HEATER_PIN_1 12 // GPIO for Zone 1 Heater Relay/SSR
#define HEATER_PIN_2 16 // GPIO for Zone 2 Heater Relay/SSR
#define HEATER_PIN_3 17 // GPIO for Zone 3 Heater Relay/SSR
const int heaterPins[3] = {HEATER_PIN_1, HEATER_PIN_2, HEATER_PIN_3};
// PWM Channels (0, 1, 2)
const int HEATER_PWM_CHAN_1 = 0;
const int HEATER_PWM_CHAN_2 = 1;
const int HEATER_PWM_CHAN_3 = 2;
const int heaterPWMChannels[3] = {HEATER_PWM_CHAN_1, HEATER_PWM_CHAN_2, HEATER_PWM_CHAN_3};
const int PWM_FREQ = 5000; // 5 kHz PWM frequency for SSRs
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)
const int PWM_MAX_DUTY = 255; // Max duty cycle

// --- CAN ID Protocol (Must match Pi's can_bridge_node.py) ---
// ESP32 -> Pi (Status/Data)
#define CAN_ID_STATUS_TEMP_1   0x101 // (float) Actual Temp Zone 1
#define CAN_ID_STATUS_TEMP_2   0x102 // (float) Actual Temp Zone 2
#define CAN_ID_STATUS_TEMP_3   0x103 // (float) Actual Temp Zone 3
#define CAN_ID_STATUS_STATE_1  0x111 // (string) "OFF", "HEAT", "PID"
#define CAN_ID_STATUS_STATE_2  0x112 // (string)
#define CAN_ID_STATUS_STATE_3  0x113 // (string)
// (Motor status... add later)

// Pi -> ESP32 (Commands)
#define CAN_ID_CMD_SETPOINT_1  0x201 // (float) Setpoint Zone 1
#define CAN_ID_CMD_SETPOINT_2  0x202 // (float) Setpoint Zone 2
#define CAN_ID_CMD_SETPOINT_3  0x203 // (float) Setpoint Zone 3
#define CAN_ID_CMD_STATE       0x210 // (2-byte) [ZoneIdx, StateCode]
#define CAN_ID_CMD_MOTOR_STATE 0x220 // (string) "ON", "OFF"
#define CAN_ID_CMD_MOTOR_RPM   0x221 // (string) "RPM50.0"

// --- Global State & Control Variables ---
#define NUM_ZONES 3

// Sensor/State Data
double actualTemp[NUM_ZONES]   = {NAN, NAN, NAN}; // Use NAN for "not a number"
double targetSetpoint[NUM_ZONES] = {0.0, 0.0, 0.0};
double heaterOutput[NUM_ZONES] = {0.0, 0.0, 0.0}; // PID output variable (0-255)
String zoneState[NUM_ZONES]    = {"OFF", "OFF", "OFF"}; // The *actual* state of the controller
String commandedState[NUM_ZONES] = {"OFF", "OFF", "OFF"}; // The *last command* from the Pi

// PID Tuning (Start with gentle values, tune later)
// Kp=Aggressive, Ki=Eliminates steady-state error, Kd=Prevents overshoot
double Kp = 5.0, Ki = 0.2, Kd = 1.0;
PID* myPID[NUM_ZONES]; // Array of PID controller objects

// State Machine Logic
const float HYSTERESIS = 3.0; // User request: ~3 degrees
const unsigned long PID_DROP_TIMEOUT_MS = 5000; // 5 seconds
unsigned long pidBelowBandStartTime[NUM_ZONES] = {0, 0, 0};

// Timing
const int STATUS_REPORT_PERIOD_MS = 500; // Send CAN status every 500ms
const int CONTROL_LOOP_PERIOD_MS = 250;  // Run PID/State logic every 250ms
const unsigned long CAN_WATCHDOG_TIMEOUT_MS = 5000; // 5 seconds
unsigned long prevStatusMillis = 0;
unsigned long prevControlMillis = 0;
unsigned long lastCanMessageTime = 0; // For watchdog

// Sensor Instances
MAX6675 thermocouple1(SCK_PIN, CS1_PIN, SO_PIN);
MAX6675 thermocouple2(SCK_PIN, CS2_PIN, SO_PIN);
MAX6675 thermocouple3(SCK_PIN, CS3_PIN, SO_PIN);
MAX6675* thermocouples[NUM_ZONES] = {&thermocouple1, &thermocouple2, &thermocouple3};

// --- Helper Functions ---
void sendCanFloat(uint32_t id, float value) {
    CanFrame txFrame = {0};
    txFrame.identifier = id;
    txFrame.extd = 0;
    txFrame.data_length_code = 4; // 4 bytes for a float
    memcpy(txFrame.data, &value, 4);
    if (!ESP32Can.writeFrame(txFrame)) {
        Serial.printf("TX FAILED ID 0x%X\n", id);
    }
}

void sendCanString(uint32_t id, String str) {
    CanFrame txFrame = {0};
    txFrame.identifier = id;
    txFrame.extd = 0;
    
    // Copy string, ensuring null-terminated and max 8 bytes
    char data[8] = {0}; // Initialize all to 0 (null)
    str.toCharArray(data, 8); // Copy max 7 chars + null
    txFrame.data_length_code = str.length(); // Send only length of string
    if (txFrame.data_length_code > 8) {
      txFrame.data_length_code = 8;
    }
    memcpy(txFrame.data, data, txFrame.data_length_code);
    
    if (!ESP32Can.writeFrame(txFrame)) {
        Serial.printf("TX FAILED ID 0x%X\n", id);
    }
}

void readAllSensors() {
    for (int i = 0; i < NUM_ZONES; i++) {
        double tempC = thermocouples[i]->readCelsius();
        // The Adafruit library returns NAN on fault
        actualTemp[i] = tempC;
    }
}

void checkCanMessages() {
    CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame)) {
        // --- NEW: Reset CAN watchdog timer on *any* received message ---
        lastCanMessageTime = millis();
        
        // Serial.printf("CAN RX ID 0x%X, DLC %d\n", rxFrame.identifier, rxFrame.data_length_code);
        
        switch (rxFrame.identifier) {
            // --- Setpoint Commands ---
            case CAN_ID_CMD_SETPOINT_1:
                if (rxFrame.data_length_code == 4) memcpy(&targetSetpoint[0], rxFrame.data, 4);
                Serial.printf("RX: Setpoint Zone 1 = %.1f\n", targetSetpoint[0]);
                break;
            case CAN_ID_CMD_SETPOINT_2:
                if (rxFrame.data_length_code == 4) memcpy(&targetSetpoint[1], rxFrame.data, 4);
                Serial.printf("RX: Setpoint Zone 2 = %.1f\n", targetSetpoint[1]);
                break;
            case CAN_ID_CMD_SETPOINT_3:
                if (rxFrame.data_length_code == 4) memcpy(&targetSetpoint[2], rxFrame.data, 4);
                Serial.printf("RX: Setpoint Zone 3 = %.1f\n", targetSetpoint[2]);
                break;

            // --- State Command (New 2-Byte Protocol) ---
            case CAN_ID_CMD_STATE: {
                if (rxFrame.data_length_code == 2) {
                    // Byte 0: Zone Index (1, 2, or 3)
                    int zoneIndex = rxFrame.data[0] - 1; // Convert 1-based to 0-based
                    // Byte 1: State Code (0=OFF, 1=HEATING, 2=PID)
                    int stateCode = rxFrame.data[1];

                    if (zoneIndex >= 0 && zoneIndex < NUM_ZONES) {
                        if (stateCode == 0) commandedState[zoneIndex] = "OFF";
                        else if (stateCode == 1) commandedState[zoneIndex] = "HEATING";
                        // The Pi should not command "PID", but we'll log it if it does
                        else if (stateCode == 2) commandedState[zoneIndex] = "PID"; 
                        Serial.printf("RX: Command Zone %d -> %s\n", zoneIndex+1, commandedState[zoneIndex].c_str());
                    }
                }
                break;
            }

            // --- Motor Commands (Stubs) ---
            case CAN_ID_CMD_MOTOR_STATE: {
                char data[9] = {0};
                memcpy(data, rxFrame.data, rxFrame.data_length_code);
                String cmdStr(data);
                Serial.printf("RX: Motor Command: %s\n", cmdStr.c_str());
                // Add motor logic here
                break;
            }
            case CAN_ID_CMD_MOTOR_RPM: {
                 char data[9] = {0};
                memcpy(data, rxFrame.data, rxFrame.data_length_code);
                String cmdStr(data);
                Serial.printf("RX: Motor RPM Command: %s\n", cmdStr.c_str());
                // Add motor logic here
                break;
            }
        }
    }
}

void runControlLogic() {
    for (int i = 0; i < NUM_ZONES; i++) {
        String currentState = zoneState[i]; // The state we are *currently* in
        String command = commandedState[i]; // The state the Pi *wants* us to be in
        double temp = actualTemp[i];
        double setpoint = targetSetpoint[i];
        
        int pwmValue = 0; // Default to OFF

        // --- State Machine ---
        // Safety check: Invalid temp or setpoint forces OFF
        if (isnan(temp) || setpoint <= 0) {
            if (currentState != "OFF") Serial.printf("Zone %d: Sensor/Setpoint invalid. Forcing OFF.\n", i+1);
            zoneState[i] = "OFF";
            pidBelowBandStartTime[i] = 0;
            pwmValue = 0;
        
        } else if (currentState == "OFF") {
            pwmValue = 0;
            pidBelowBandStartTime[i] = 0;
            // --- UPDATED: Check for setpoint > 0 *before* allowing transition ---
            if (command == "HEATING" && setpoint > 0) {
                zoneState[i] = "HEATING";
                Serial.printf("Zone %d: Transitioning OFF -> HEATING (Setpoint: %.1f)\n", i+1, setpoint);
            }
            
        } else if (currentState == "HEATING") {
            pwmValue = PWM_MAX_DUTY; // Full power
            pidBelowBandStartTime[i] = 0;
            if (command == "OFF") {
                zoneState[i] = "OFF";
                Serial.printf("Zone %d: Transitioning HEATING -> OFF (Commanded)\n", i+1);
            } else if (temp >= (setpoint - HYSTERESIS)) {
                zoneState[i] = "PID";
                Serial.printf("Zone %d: Temp (%.1f) in band. Transitioning HEATING -> PID\n", i+1, temp);
                myPID[i]->SetMode(AUTOMATIC); // Turn on PID computation
            }

        } else if (currentState == "PID") {
            if (command == "OFF") {
                zoneState[i] = "OFF";
                Serial.printf("Zone %d: Transitioning PID -> OFF (Commanded)\n", i+1);
                myPID[i]->SetMode(MANUAL); // Turn off PID computation
                pwmValue = 0;
            } else {
                // Check for temperature drop
                if (temp < (setpoint - HYSTERESIS)) {
                    if (pidBelowBandStartTime[i] == 0) {
                        pidBelowBandStartTime[i] = millis();
                        Serial.printf("Zone %d: Temp (%.1f) dropped below PID band. Starting timer.\n", i+1, temp);
                        pwmValue = PWM_MAX_DUTY; // Full power to recover
                    } else if (millis() - pidBelowBandStartTime[i] > PID_DROP_TIMEOUT_MS) {
                        Serial.printf("Zone %d: Temp still low. Transitioning PID -> HEATING\n", i+1);
                        zoneState[i] = "HEATING";
                        myPID[i]->SetMode(MANUAL); // Turn off PID computation
                        pwmValue = PWM_MAX_DUTY;
                    } else {
                        // Still in dropout timer, keep heater on
                        pwmValue = PWM_MAX_DUTY;
                    }
                } else {
                    // Temp is good, reset timer
                    pidBelowBandStartTime[i] = 0;
                    
                    // --- NEW: Use true PID logic ---
                    // The PID library needs inputs (current temp), setpoint,
                    // and a variable to write its output to.
                    // We must update the setpoint for the PID object
                    // myPID[i]->SetTunings(Kp, Ki, Kd); // Can also update tunings here
                    
                    // Compute() returns true if a new output was computed
                    bool computed = myPID[i]->Compute(); 
                    if(computed) {
                        pwmValue = (int)heaterOutput[i]; // Use the output from the PID lib
                        Serial.printf("Zone %d: PID Compute. Temp: %.1f, Target: %.1f, Output: %d\n", i+1, temp, setpoint, pwmValue);
                    }
                }
            }
        }
        
        // --- Control Physical Heater Pin ---
        // Use ledcWrite to set the PWM duty cycle (0-255)
        ledcWrite(heaterPWMChannels[i], pwmValue);
    }
}

void checkCanWatchdog() {
    if (millis() - lastCanMessageTime > CAN_WATCHDOG_TIMEOUT_MS) {
        // CAN connection lost!
        Serial.println("CRITICAL ERROR: CAN Watchdog Timeout! Forcing all heaters OFF.");
        for (int i = 0; i < NUM_ZONES; i++) {
            zoneState[i] = "OFF"; // Force state to OFF
            commandedState[i] = "OFF"; // Reset command
            ledcWrite(heaterPWMChannels[i], 0); // Turn off heater
        }
        // Reset timer to prevent spamming logs
        lastCanMessageTime = millis(); 
    }
}

void sendAllStatus() {
    Serial.print("CAN TX: ");
    for (int i = 0; i < NUM_ZONES; i++) {
        // Send Temperature
        sendCanFloat(CAN_ID_STATUS_TEMP_1 + i, actualTemp[i]);
        Serial.printf("Z%d Temp: %.1f | ", i+1, actualTemp[i]);
        
        // Send State
        sendCanString(CAN_ID_STATUS_STATE_1 + i, zoneState[i]);
        Serial.printf("Z%d State: %s | ", i+1, zoneState[i].c_str());
    }
    Serial.println();
}

// --- ARDUINO SKETCH ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("--- Frankenmolder ESP32 Controller (v2 - True PID) ---");

    // 1. Configure Heater Pins as PWM
    for (int i = 0; i < NUM_ZONES; i++) {
        // --- FIX: Use ledcAttach (more compatible) ---
        //ledcSetup(heaterPWMChannels[i], PWM_FREQ, PWM_RESOLUTION);
      //  ledcAttach(heaterPins[i], heaterPWMChannels[i]); // Use ledcAttach
       // ledcWrite(heaterPWMChannels[i], 0); // Start OFF
    }
    Serial.println("Heater PWM pins initialized.");

    // 2. Configure CAN Bus
    ESP32Can.setPins(CAN_TX, CAN_RX);
    if(ESP32Can.begin(TWAI_SPEED_500KBPS)) {
        Serial.println("CAN bus started successfully!");
    } else {
        Serial.println("CRITICAL ERROR: CAN bus failed!");
        while (1) delay(1000);
    }
    // Initialize watchdog timer
    lastCanMessageTime = millis();

    // 3. Configure PID Controllers
    for (int i = 0; i < NUM_ZONES; i++) {
        // We pass pointers to the global variables
        myPID[i] = new PID(&actualTemp[i], &heaterOutput[i], &targetSetpoint[i], Kp, Ki, Kd, DIRECT);
        // --- NEW: Set PID output limits to 0-255 for 8-bit PWM ---
        myPID[i]->SetOutputLimits(0, PWM_MAX_DUTY);
        // Start in MANUAL mode (off) until we enter PID state
        myPID[i]->SetMode(MANUAL);
    }
    Serial.println("PID controllers initialized.");

    // 4. Initialize Sensors
    delay(500); // Wait for sensors to stabilize
    readAllSensors();
    Serial.println("Initialization complete.");
}

void loop() {
    unsigned long currentMillis = millis();

    // 1. Always read sensors
    readAllSensors();

    // 2. Always check for incoming commands
    checkCanMessages();

    // 3. NEW: Check for CAN connection loss
    checkCanWatchdog();
    
    // 4. Run control logic periodically
    if (currentMillis - prevControlMillis >= CONTROL_LOOP_PERIOD_MS) {
        prevControlMillis = currentMillis;
        runControlLogic();
    }

    // 5. Send status reports periodically
    if (currentMillis - prevStatusMillis >= STATUS_REPORT_PERIOD_MS) {
        prevStatusMillis = currentMillis;
        sendAllStatus();
    }
    
    // Short delay to prevent loop from spinning too fast
    delay(10); 
}