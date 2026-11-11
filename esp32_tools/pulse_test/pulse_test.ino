/**
 * Frankenmolder - servo drive pulse test (standalone, NOT the main firmware)
 *
 * Sends a plain 50% duty square wave on the pulse pin, so you can check whether the
 * drive counts pulses (drive display d9-Fr = frequency in kHz, d2-CP = pulse count)
 * without FastAccelStepper in the way.
 *
 * Same wiring as the main firmware:
 *   GPIO32 pulse -> transistor -> drive pin 43 (PULSE-), +24 V on pin 35
 *   GPIO33 dir   -> transistor -> drive pin 39 (SIGN-)
 *   GPIO26 SON   -> transistor -> drive pin 9  (DI1), +24 V on pin 11 (COM+)
 *
 * Serial monitor at 115200, line ending "Newline". Commands:
 *   f 1000   set pulse frequency in Hz (0 = stop)    e.g. f 100, f 1000, f 10000
 *   d 0/1    direction pin LOW / HIGH
 *   s 0/1    SON off / on
 *   ?        show current settings
 *
 * Starts with SON ON, direction LOW and 1000 Hz (6 rpm at 10000 pulses/rev).
 * Motor uncoupled from the screw while testing. Re-flash the main firmware afterwards.
 */

#include <Arduino.h>

#define PULSE_PIN 32
#define DIR_PIN   33
#define SON_PIN   26

const uint8_t RES_BITS = 8;  // 50% duty = 128
uint32_t freqHz = 1000;
bool dirHigh = false;
bool sonOn = true;

void applyPulse() {
    if (freqHz == 0) {
        ledcWrite(PULSE_PIN, 0);                          // output held LOW = transistor off
    } else {
        ledcChangeFrequency(PULSE_PIN, freqHz, RES_BITS);
        ledcWrite(PULSE_PIN, 1 << (RES_BITS - 1));        // 50% duty
    }
}

void show() {
    Serial.printf("SON %s | DIR %s | pulse %lu Hz (50%% duty) on GPIO%d\n",
                  sonOn ? "ON" : "OFF", dirHigh ? "HIGH" : "LOW", (unsigned long)freqHz, PULSE_PIN);
    Serial.println("Drive display: d9-Fr should read about the same in kHz, d2-CP should count up.");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- Servo pulse test (50% duty square wave) ---");

    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
    pinMode(SON_PIN, OUTPUT);
    digitalWrite(SON_PIN, HIGH);                          // SON on
    delay(100);                                           // drive wants >= 50 ms before pulses

    if (!ledcAttach(PULSE_PIN, freqHz, RES_BITS)) {
        Serial.println("ERROR: ledcAttach failed on the pulse pin");
    }
    applyPulse();
    show();
    Serial.println("Commands: f <Hz>, d 0|1, s 0|1, ?");
}

void loop() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    char cmd = line.charAt(0);
    long val = line.length() > 1 ? line.substring(1).toInt() : 0;

    switch (cmd) {
        case 'f':
            freqHz = (uint32_t)constrain(val, 0L, 100000L);
            if (freqHz > 0 && freqHz < 20) freqHz = 20;   // LEDC lower limit at 8 bits
            applyPulse();
            break;
        case 'd':
            dirHigh = val != 0;
            digitalWrite(DIR_PIN, dirHigh ? HIGH : LOW);
            break;
        case 's':
            sonOn = val != 0;
            digitalWrite(SON_PIN, sonOn ? HIGH : LOW);
            break;
        case '?':
            break;
        default:
            Serial.println("Commands: f <Hz>, d 0|1, s 0|1, ?");
            return;
    }
    show();
}
