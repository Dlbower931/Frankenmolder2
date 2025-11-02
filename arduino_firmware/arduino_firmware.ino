// --- USING OFFICIAL ADAFRUIT MAX6675 LIBRARY (SOFTWARE SPI) ---
// This strategy avoids all hardware SPI bus conflicts.
//
// YOU MUST INSTALL THE OFFICIAL LIBRARY:
// In Arduino IDE Library Manager, search for and install:
// "Adafruit MAX6675 Library"
//
// YOU MUST UNINSTALL the generic <max6675.h> library to avoid conflicts.

#include <Arduino.h> 
// We NO LONGER include <SPI.h> because this is Software SPI
#include <ESP32-TWAI-CAN.hpp>

// --- ADAFRUIT LIBRARY ---
#include <max6675.h> 

// --- CAN PIN DEFINITIONS (From your working test code) ---
#define CAN_TX 5 // Connects to Transceiver's TXD/D pin
#define CAN_RX 4 // Connects to Transceiver's RXD/R pin

// --- MAX6675 SOFTWARE SPI PIN DEFINITIONS (AVOIDING PINS 4 & 5) ---
// These can be any GPIO pins, but we'll stick to our safe ones.
#define SCK_PIN  18  // Shared SPI Clock
#define SO_PIN   19  // Shared SPI Data Out (MISO)
#define CS1_PIN  13  // Chip Select for Zone 1 (Safe Pin)
#define CS2_PIN  14  // Chip Select for Zone 2 (Safe Pin)
#define CS3_PIN  27  // Chip Select for Zone 3 (Safe Pin)

// --- CAN BUS ID DEFINITIONS (Status only) ---
#define CAN_ID_STATUS_TEMP_1    0x101 // ESP32 -> Pi (Actual Temp Zone 1)
#define CAN_ID_STATUS_TEMP_2    0x102 // ESP32 -> Pi (Actual Temp Zone 2)
#define CAN_ID_STATUS_TEMP_3    0x103 // ESP32 -> Pi (Actual Temp Zone 3)

// --- GLOBAL STATE & INSTANCES ---
float actualTemp[3] = {0.0, 0.0, 0.0};
const int STATUS_REPORT_PERIOD_MS = 500; 
unsigned long previousMillis = 0;

// --- ADAFRUIT MAX6675 INSTANCES (Using Software SPI Constructors) ---
// Format: (sclk_pin, cs_pin, miso_pin)
MAX6675 thermocouple1(SCK_PIN, CS1_PIN, SO_PIN);
MAX6675 thermocouple2(SCK_PIN, CS2_PIN, SO_PIN);
MAX6675 thermocouple3(SCK_PIN, CS3_PIN, SO_PIN);


// --- CORE FUNCTIONS ---

// Function to read the temperature from the actual sensor
void readAllSensors() {
    // Read the temperatures using the Adafruit library
    actualTemp[0] = thermocouple1.readCelsius();
    actualTemp[1] = thermocouple2.readCelsius();
    actualTemp[2] = thermocouple3.readCelsius();
}

/**
 * @brief Sends a single float value (temperature) over the CAN bus.
 * @param id The CAN arbitration ID (e.g., CAN_ID_STATUS_TEMP_1).
 * @param value The 32-bit float value to send.
 */
void sendCanStatus(uint32_t id, float value) {
    CanFrame txFrame = {0};
    
    // Set the Frame properties
    txFrame.identifier = id; 
    txFrame.extd = 0;          // Standard 11-bit ID
    txFrame.data_length_code = 4; // Use 4 bytes for the float

    // Convert the float into 4 bytes for the CAN payload
    memcpy(txFrame.data, &value, 4); 
    
    // Write Frame to the Bus
    if(ESP32Can.writeFrame(txFrame)) {
        // Only print success if the reading is valid
        if (!isnan(value)) {
            Serial.printf("TX SUCCESS ID 0x%X: %.2f C\n", id, value);
        }
    } else {
        Serial.printf("TX FAILED ID 0x%X\n", id);
    }
}

// --- ARDUINO SKETCH FUNCTIONS ---

void setup() {
    Serial.begin(115200);
    delay(1000); 
    Serial.println("--- EXTRUDER STATUS TX (ADAFRUIT LIB / SOFTWARE SPI) ---");
    Serial.printf("CAN Pins: TX=%d, RX=%d\n", CAN_TX, CAN_RX);
    Serial.printf("SPI Pins: SCK=%d, SO=%d, CS1=%d, CS2=%d, CS3=%d\n", SCK_PIN, SO_PIN, CS1_PIN, CS2_PIN, CS3_PIN);
    
    // 1. Configure CAN Pins and Start Bus
    ESP32Can.setPins(CAN_TX, CAN_RX);
    if(ESP32Can.begin(TWAI_SPEED_500KBPS)) { 
        Serial.println("CAN bus started successfully at 500kbit/s!");
    } else {
        Serial.println("CRITICAL ERROR: CAN bus failed to start! Check Transceiver wiring.");
        while (1) { delay(1000); } 
    }
    
    // 2. Initialize Thermocouples
    // NO MORE pinMode() or SPI.begin()! The Adafruit library handles it all.
    Serial.println("Waiting for sensor stabilization...");
    delay(500); // Wait 0.5s for sensors to stabilize before first read.
    Serial.println("Initialization complete.");
}

void loop() {
    // Read all sensor values in every loop cycle
    readAllSensors();
    
    // --- Print the current raw readings for console debug ---
    Serial.print("READINGS: ");
    
    for (int i = 0; i < 3; i++) {
        // The Adafruit library returns NaN (Not a Number) on fault.
        if (isnan(actualTemp[i])) { 
            Serial.printf("Z%d=FAULT | ", i + 1);
        } else {
            Serial.printf("Z%d=%.2fC | ", i + 1, actualTemp[i]);
        }
    }
    Serial.println();
    // -----------------------------------------------------------
    
    unsigned long currentMillis = millis();

    // Check if it's time to send status updates over CAN
    if (currentMillis - previousMillis >= STATUS_REPORT_PERIOD_MS) {
        previousMillis = currentMillis;

        // 1. Send Zone 1 Temperature Status
        sendCanStatus(CAN_ID_STATUS_TEMP_1, actualTemp[0]);

        // 2. Send Zone 2 Temperature Status
        sendCanStatus(CAN_ID_STATUS_TEMP_2, actualTemp[1]);

        // 3. Send Zone 3 Temperature Status
        sendCanStatus(CAN_ID_STATUS_TEMP_3, actualTemp[2]);
        
        Serial.println("---");
    }

    // CRITICAL: Delay to ensure MAX6675 meets its 220ms minimum conversion time.
    delay(250); 
}


