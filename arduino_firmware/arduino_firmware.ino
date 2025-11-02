/**
 * Frankenmolder - ESP32 Main Control Unit
 *
 * This code runs on the ESP32 and performs all real-time tasks:
 * 1. Reads 3x MAX6675 temperature sensors via Software SPI.
 * 2. Receives Setpoint and State Commands from the Pi over CAN bus.
 * 3. Runs a State Machine (OFF, HEATING, PID) for each of the 3 zones.
 * 4. Runs a PID controller for each zone.
 * 5. Controls 3x Heater GPIO pins.
 * 6. Publishes actual Temperature and actual State back to the Pi over CAN.
 * * REQUIRED LIBRARIES:
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
#define CAN_TX 5
#define CAN_RX 4
// MAX6675 Software SPI
#define SCK_PIN 18
#define SO_PIN  19
#define CS1_PIN 13
#define CS2_PIN 14
#define CS3_PIN 27
// Heater Control (NEW - Choose your pins)
#define HEATER_PIN_1 12 // GPIO for Zone 1 Heater Relay/SSR
#define HEATER_PIN_2 16 // GPIO for Zone 2 Heater Relay/SSR
#define HEATER_PIN_3 17 // GPIO for Zone 3 Heater Relay/SSR
const int heaterPins[3] = {HEATER_PIN_1, HEATER_PIN_2, HEATER_PIN_3};

// --- CAN ID Protocol ---
// ESP32 -> Pi (Status/Data)
#define CAN_ID_STATUS_TEMP_1   0x101 // (float) Actual Temp Zone 1
#define CAN_ID_STATUS_TEMP_2   0x102 // (float) Actual Temp Zone 2
#define CAN_ID_STATUS_TEMP_3   0x103 // (float) Actual Temp Zone 3
#define CAN_ID_STATUS_STATE_1  0x111 // (string) "OFF", "HEAT", "PID"
#define CAN_ID_STATUS_STATE_2  0x112 // (string)
#define CAN_ID_STATUS_STATE_3  0x113 // (string)

// Pi -> ESP32 (Commands)
#define CAN_ID_CMD_SETPOINT_1  0x201 // (float) Setpoint Zone 1
#define CAN_ID_CMD_SETPOINT_2  0x202 // (float) Setpoint Zone 2
#define CAN_ID_CMD_SETPOINT_3  0x203 // (float) Setpoint Zone 3
#define CAN_ID_CMD_STATE       0x210 // (string) "zone1:HEATING", "zone2:OFF", etc.
#define CAN_ID_CMD_MOTOR_STATE 0x220 // (string) "ON", "OFF"
#define CAN_ID_CMD_MOTOR_RPM   0x221 // (string) "RPM50.0"

// --- Global State & Control Variables ---
#define NUM_ZONES 3

// Sensor/State Data
double actualTemp[NUM_ZONES]   = {0.0, 0.0, 0.0};
double targetSetpoint[NUM_ZONES] = {0.0, 0.0, 0.0};
double heaterOutput[NUM_ZONES] = {0.0, 0.0, 0.0}; // PID output variable (0-255)
String zoneState[NUM_ZONES]    = {"OFF", "OFF", "OFF"};
String commandedState[NUM_ZONES] = {"OFF", "OFF", "OFF"}; // From Pi

// PID Tuning (Start with gentle values, tune later)
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
PID* myPID[NUM_ZONES]; // Array of PID controller objects

// State Machine Logic
const float HYSTERESIS = 2.0;
const unsigned long PID_DROP_TIMEOUT_MS = 5000; // 5 seconds
unsigned long pidBelowBandStartTime[NUM_ZONES] = {0, 0, 0};

// Timing
const int STATUS_REPORT_PERIOD_MS = 500; // Send CAN status every 500ms
const int CONTROL_LOOP_PERIOD_MS = 250;  // Run PID/State logic every 250ms
unsigned long prevStatusMillis = 0;
unsigned long prevControlMillis = 0;

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
    char data[8] = {0};
    str.toCharArray(data, 8);
    txFrame.data_length_code = strlen(data); // Send only length of string
    memcpy(txFrame.data, data, txFrame.data_length_code);
    
    if (!ESP32Can.writeFrame(txFrame)) {
        Serial.printf("TX FAILED ID 0x%X\n", id);
    }
}

void readAllSensors() {
    for (int i = 0; i < NUM_ZONES; i++) {
        double tempC = thermocouples[i]->readCelsius();
        if (isnan(tempC)) {
            // Use last known good temp? For now, just mark as NaN
            actualTemp[i] = NAN; 
        } else {
            actualTemp[i] = tempC;
        }
    }
}

void checkCanMessages() {
    CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame)) {
        Serial.printf("CAN RX ID 0x%X, DLC %d\n", rxFrame.identifier, rxFrame.data_length_code);
        
        switch (rxFrame.identifier) {
            // --- Setpoint Commands ---
            case CAN_ID_CMD_SETPOINT_1:
                if (rxFrame.data_length_code == 4) memcpy(&targetSetpoint[0], rxFrame.data, 4);
                break;
            case CAN_ID_CMD_SETPOINT_2:
                if (rxFrame.data_length_code == 4) memcpy(&targetSetpoint[1], rxFrame.data, 4);
                break;
            case CAN_ID_CMD_SETPOINT_3:
                if (rxFrame.data_length_code == 4) memcpy(&targetSetpoint[2], rxFrame.data, 4);
                break;

            // --- State Command ---
            case CAN_ID_CMD_STATE: {
                char data[9] = {0}; // 8 chars + null terminator
                memcpy(data, rxFrame.data, rxFrame.data_length_code);
                String cmdStr(data); // e.g., "zone1:HEATING"
                
                int colonIndex = cmdStr.indexOf(':');
                if (colonIndex > 0) {
                    String zone = cmdStr.substring(0, colonIndex); // "zone1"
                    String state = cmdStr.substring(colonIndex + 1); // "HEATING"
                    
                    int zoneIndex = -1;
                    if (zone == "zone1") zoneIndex = 0;
                    else if (zone == "zone2") zoneIndex = 1;
                    else if (zone == "zone3") zoneIndex = 2;
                    
                    if (zoneIndex != -1) {
                        if (state == "OFF" || state == "HEATING") {
                            Serial.printf("Received Command: Zone %d -> %s\n", zoneIndex+1, state.c_str());
                            commandedState[zoneIndex] = state;
                        }
                    }
                }
                break;
            }

            // --- Motor Commands (Stubs) ---
            case CAN_ID_CMD_MOTOR_STATE: {
                char data[9] = {0};
                memcpy(data, rxFrame.data, rxFrame.data_length_code);
                String cmdStr(data);
                Serial.printf("Motor Command: %s\n", cmdStr.c_str());
                // Add motor logic here
                break;
            }
            case CAN_ID_CMD_MOTOR_RPM: {
                 char data[9] = {0};
                memcpy(data, rxFrame.data, rxFrame.data_length_code);
                String cmdStr(data);
                Serial.printf("Motor RPM Command: %s\n", cmdStr.c_str());
                // Add motor logic here
                break;
            }
        }
    }
}

void runControlLogic() {
    for (int i = 0; i < NUM_ZONES; i++) {
        String currentState = zoneState[i];
        String command = commandedState[i];
        double temp = actualTemp[i];
        double setpoint = targetSetpoint[i];
        
        bool heaterOn = false; // Default to OFF

        // --- State Machine ---
        // Safety check: Invalid temp or setpoint forces OFF
        if (isnan(temp) || setpoint <= 0) {
            if (currentState != "OFF") Serial.printf("Zone %d: Sensor/Setpoint invalid. Forcing OFF.\n", i+1);
            zoneState[i] = "OFF";
            pidBelowBandStartTime[i] = 0;
            heaterOn = false;
        
        } else if (currentState == "OFF") {
            heaterOn = false;
            pidBelowBandStartTime[i] = 0;
            if (command == "HEATING" && setpoint > 0) {
                zoneState[i] = "HEATING";
                Serial.printf("Zone %d: Transitioning OFF -> HEATING\n", i+1);
            }
            
        } else if (currentState == "HEATING") {
            heaterOn = true;
            pidBelowBandStartTime[i] = 0;
            if (command == "OFF") {
                zoneState[i] = "OFF";
                Serial.printf("Zone %d: Transitioning HEATING -> OFF (Commanded)\n", i+1);
            } else if (temp >= (setpoint - HYSTERESIS)) {
                zoneState[i] = "PID";
                Serial.printf("Zone %d: Temp (%.1f) in band. Transitioning HEATING -> PID\n", i+1, temp);
            }

        } else if (currentState == "PID") {
            if (command == "OFF") {
                zoneState[i] = "OFF";
                Serial.printf("Zone %d: Transitioning PID -> OFF (Commanded)\n", i+1);
            } else {
                // Check for temperature drop
                if (temp < (setpoint - HYSTERESIS)) {
                    if (pidBelowBandStartTime[i] == 0) {
                        pidBelowBandStartTime[i] = millis();
                        Serial.printf("Zone %d: Temp (%.1f) dropped below PID band. Starting timer.\n", i+1, temp);
                        heaterOn = true; // Keep heater on
                    } else if (millis() - pidBelowBandStartTime[i] > PID_DROP_TIMEOUT_MS) {
                        Serial.printf("Zone %d: Temp still low. Transitioning PID -> HEATING\n", i+1);
                        zoneState[i] = "HEATING";
                        heaterOn = true;
                    } else {
                        // Still in dropout timer, keep heater on
                        heaterOn = true;
                    }
                } else {
                    // Temp is good, reset timer
                    pidBelowBandStartTime[i] = 0;
                    // Run PID logic (simple ON/OFF for now)
                    if (temp < setpoint) {
                        heaterOn = true;
                    } else {
                        heaterOn = false;
                    }
                }
            }
        }
        
        // --- Control Physical Heater Pin ---
        digitalWrite(heaterPins[i], heaterOn ? HIGH : LOW);
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
    Serial.println("--- Frankenmolder ESP32 Controller ---");

    // 1. Configure Heater Pins
    for (int i = 0; i < NUM_ZONES; i++) {
        pinMode(heaterPins[i], OUTPUT);
        digitalWrite(heaterPins[i], LOW); // Start OFF
    }
    Serial.println("Heater pins initialized.");

    // 2. Configure CAN Bus
    ESP32Can.setPins(CAN_TX, CAN_RX);
    if(ESP32Can.begin(TWAI_SPEED_500KBPS)) {
        Serial.println("CAN bus started successfully!");
    } else {
        Serial.println("CRITICAL ERROR: CAN bus failed!");
        while (1) delay(1000);
    }

    // 3. Configure PID Controllers
    for (int i = 0; i < NUM_ZONES; i++) {
        // We pass pointers to the global variables
        myPID[i] = new PID(&actualTemp[i], &heaterOutput[i], &targetSetpoint[i], Kp, Ki, Kd, DIRECT);
        myPID[i]->SetMode(AUTOMATIC);
        // This simple state machine uses ON/OFF (0 or 1)
        myPID[i]->SetOutputLimits(0, 1); 
        // We will read heaterOutput[i] and use digitalWrite()
        // (Note: This sketch uses simple on/off, not the PID output variable.
        // To use true PID, you'd need to set limits 0-255 and use analogWrite/PWM)
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
    
    // 3. Run control logic periodically
    if (currentMillis - prevControlMillis >= CONTROL_LOOP_PERIOD_MS) {
        prevControlMillis = currentMillis;
        runControlLogic();
    }

    // 4. Send status reports periodically
    if (currentMillis - prevStatusMillis >= STATUS_REPORT_PERIOD_MS) {
        prevStatusMillis = currentMillis;
        sendAllStatus();
    }
    
    // Short delay to prevent loop from spinning too fast
    delay(10); 
}
