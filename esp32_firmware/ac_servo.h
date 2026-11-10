#pragma once
/**
 * Frankenmolder - AC servo test control (XP200 drive, pulse + direction mode)
 *
 * Generates step pulses + direction for the servo drive with ramps, exact pulse
 * counts for position moves, and safety stops. Driven by its own CAN IDs, separate
 * from the old hobby-servo motor commands (0x220/0x221), which are left as they are.
 *
 * REQUIRED LIBRARY: FastAccelStepper (by gin66). Compile-tested with v2.0.1 on ESP32 core 3.3.12.
 *
 * ---------------------------------------------------------------------------
 * WIRING (drive CN1)
 *   SON and COM+ are hard-wired (constant 24 V), so the drive is enabled whenever it
 *   is powered: the motor holds position and ANY pulse will move it.
 *
 *   GPIO32 (pulse) / GPIO33 (dir) are 3.3 V. The drive's opto inputs want 8-15 mA, so
 *   feed them through a 5 V differential line driver (e.g. AM26LS31):
 *     driver out A / A-bar -> PULS+ (5V) pin 25 / PULS- pin 9
 *     driver out B / B-bar -> SIGN+ (5V) pin 24 / SIGN- pin 8
 *   Common ground between the ESP32 and the line driver.
 *
 *   Optional alarm input: drive ALM (pin 5) / DO COM (pin 23) through an optocoupler
 *   or open-collector to GPIO34 with an external 10k pull-up to 3.3 V (GPIO34 has no
 *   internal pull-up). ALM conducts when there is NO alarm -> GPIO34 LOW = OK.
 *   Set SERVO_ALARM_INPUT_ENABLED to 1 once it is wired.
 *
 * Drive parameters to check: Pr004 = 0 (position mode), Pr035 = 0 (pulse + direction),
 * Pr029/Pr030 electronic gear (default 10000 pulses/motor rev; Pr029=5 -> 2000/rev).
 *
 * ---------------------------------------------------------------------------
 * CAN PROTOCOL (all little-endian, standard 11-bit IDs)
 *   Pi -> ESP32
 *   0x230 SERVO_CMD    [u8 cmd]                  1=RUN 2=STOP(ramp) 3=ESTOP(immediate) 4=ZERO position
 *   0x231 SERVO_SPEED  [f32 target_hz][f32 accel] target_hz signed (negative = reverse),
 *                                                accel in Hz per second. Applied live while running.
 *   0x232 SERVO_MOVE   [i32 pulses][f32 hz]       relative move (signed pulses) at hz, from standstill only
 *   0x200 HEARTBEAT    (existing) - the servo stops if no CAN frame arrives for 3 s while moving.
 *
 *   ESP32 -> Pi (every 100 ms)
 *   0x130 SERVO_STATUS [u8 state][u8 flags][2 reserved][f32 actual_hz signed]
 *                      state: 0=IDLE 1=RUNNING 2=MOVING 3=STOPPING 4=FAULT
 *                      flags: bit0 alarm input active, bit1 stopped by heartbeat watchdog,
 *                             bit2 at target speed, bit3 alarm input enabled
 *   0x131 SERVO_POS    [i32 position pulses][i32 target_hz echo]
 */

#include <FastAccelStepper.h>

// --- Pins ---
#define SERVO_PULSE_PIN 32
#define SERVO_DIR_PIN   33
#define SERVO_ALARM_PIN 34
#define SERVO_ALARM_INPUT_ENABLED 0   // 1 once ALM is wired to GPIO34

// --- CAN IDs ---
#define CAN_ID_CMD_SERVO        0x230
#define CAN_ID_CMD_SERVO_SPEED  0x231
#define CAN_ID_CMD_SERVO_MOVE   0x232
#define CAN_ID_STATUS_SERVO     0x130
#define CAN_ID_STATUS_SERVO_POS 0x131

enum : uint8_t { SERVO_CMD_RUN = 1, SERVO_CMD_STOP = 2, SERVO_CMD_ESTOP = 3, SERVO_CMD_ZERO = 4 };
enum : uint8_t { SERVO_IDLE = 0, SERVO_RUNNING = 1, SERVO_MOVING = 2, SERVO_STOPPING = 3,
                 SERVO_FAULT = 4 };

// --- Limits ---
const uint32_t SERVO_MAX_HZ = 200000;           // ESP32 + FastAccelStepper practical limit
const uint32_t SERVO_MIN_ACCEL = 100;           // Hz/s
const uint32_t SERVO_MAX_ACCEL = 2000000;       // Hz/s
const uint32_t SERVO_SAFETY_DECEL = 100000;     // Hz/s used for watchdog stops
const unsigned long SERVO_HEARTBEAT_TIMEOUT_MS = 3000;
const unsigned long SERVO_STATUS_PERIOD_MS = 100;

// --- State ---
static FastAccelStepperEngine servoEngine = FastAccelStepperEngine();
static FastAccelStepper* acServo = nullptr;
static float servoTargetHz = 0.0f;        // signed
static uint32_t servoAccel = 20000;       // Hz/s
static uint8_t servoMode = SERVO_IDLE;    // what was last commanded: IDLE / RUNNING / MOVING
static bool servoFault = false;           // latched by the alarm input, cleared by STOP
static bool servoWatchdogTrip = false;    // set when the heartbeat watchdog stopped us
static unsigned long servoLastRxMs = 0;
static unsigned long servoPrevStatusMs = 0;

inline void acServoNoteRx() { servoLastRxMs = millis(); }

static bool acServoAlarmActive() {
#if SERVO_ALARM_INPUT_ENABLED
    return digitalRead(SERVO_ALARM_PIN) == HIGH;  // ALM conducts (LOW) when there is no alarm
#else
    return false;
#endif
}

static uint32_t servoClampHz(float hz) {
    hz = fabsf(hz);
    if (hz > SERVO_MAX_HZ) hz = SERVO_MAX_HZ;
    return (uint32_t)hz;
}

static uint32_t servoClampAccel(float a) {
    if (a < SERVO_MIN_ACCEL) a = SERVO_MIN_ACCEL;
    if (a > SERVO_MAX_ACCEL) a = SERVO_MAX_ACCEL;
    return (uint32_t)a;
}

// Start or update continuous running at servoTargetHz (ramped).
static void acServoApplyRun() {
    uint32_t hz = servoClampHz(servoTargetHz);
    acServo->setAcceleration(servoAccel);
    if (hz < 1) {                        // target 0 -> ramp to a stop
        acServo->stopMove();
        servoMode = SERVO_IDLE;
        return;
    }
    acServo->setSpeedInHz(hz);
    if (servoTargetHz >= 0) acServo->runForward(); else acServo->runBackward();
    servoMode = SERVO_RUNNING;
}

static void acServoSafetyStop() {
    acServo->setAcceleration(max(servoAccel, SERVO_SAFETY_DECEL));
    acServo->applySpeedAcceleration();
    acServo->stopMove();
    servoMode = SERVO_IDLE;
}

void acServoSetup() {
    servoEngine.init();
    acServo = servoEngine.stepperConnectToPin(SERVO_PULSE_PIN);
    if (!acServo) {
        Serial.println("AC servo: FAILED to attach pulse generator to GPIO32");
        return;
    }
    acServo->setDirectionPin(SERVO_DIR_PIN);
    acServo->setAutoEnable(false);       // SON is hard-wired on
    acServo->setSpeedInHz(1000);
    acServo->setAcceleration(servoAccel);
#if SERVO_ALARM_INPUT_ENABLED
    pinMode(SERVO_ALARM_PIN, INPUT);     // external 10k pull-up required
#endif
    servoLastRxMs = millis();
    Serial.printf("AC servo: pulse GPIO%d, dir GPIO%d ready (max %lu Hz)\n",
                  SERVO_PULSE_PIN, SERVO_DIR_PIN, (unsigned long)SERVO_MAX_HZ);
}

// Returns true if the frame was a servo command.
bool acServoHandleFrame(const CanFrame& f) {
    if (f.identifier != CAN_ID_CMD_SERVO && f.identifier != CAN_ID_CMD_SERVO_SPEED &&
        f.identifier != CAN_ID_CMD_SERVO_MOVE) {
        return false;
    }
    if (!acServo) return true;

    if (f.identifier == CAN_ID_CMD_SERVO && f.data_length_code >= 1) {
        switch (f.data[0]) {
            case SERVO_CMD_RUN:
                if (servoFault) { Serial.println("AC servo: RUN refused - alarm latched (send STOP to clear)"); break; }
                servoWatchdogTrip = false;
                acServoApplyRun();
                Serial.printf("AC servo: RUN at %.0f Hz, accel %lu Hz/s\n", servoTargetHz, (unsigned long)servoAccel);
                break;
            case SERVO_CMD_STOP:
                acServo->stopMove();
                servoMode = SERVO_IDLE;
                if (!acServoAlarmActive()) servoFault = false;
                Serial.println("AC servo: STOP (ramp)");
                break;
            case SERVO_CMD_ESTOP:
                acServo->forceStop();
                servoMode = SERVO_IDLE;
                Serial.println("AC servo: E-STOP (pulses cut immediately)");
                break;
            case SERVO_CMD_ZERO:
                if (!acServo->isRunning()) acServo->setCurrentPosition(0);
                break;
        }
    } else if (f.identifier == CAN_ID_CMD_SERVO_SPEED && f.data_length_code >= 8) {
        float hz, accel;
        memcpy(&hz, f.data, 4);
        memcpy(&accel, f.data + 4, 4);
        if (isnan(hz) || isnan(accel)) return true;
        servoTargetHz = constrain(hz, -(float)SERVO_MAX_HZ, (float)SERVO_MAX_HZ);
        servoAccel = servoClampAccel(accel);
        if (servoMode == SERVO_RUNNING) acServoApplyRun();   // live speed / direction change
        Serial.printf("AC servo: target %.0f Hz, accel %lu Hz/s\n", servoTargetHz, (unsigned long)servoAccel);
    } else if (f.identifier == CAN_ID_CMD_SERVO_MOVE && f.data_length_code >= 8) {
        int32_t pulses;
        float hz;
        memcpy(&pulses, f.data, 4);
        memcpy(&hz, f.data + 4, 4);
        if (servoFault || acServo->isRunning()) {
            Serial.println("AC servo: MOVE ignored (running or alarm)");
            return true;
        }
        uint32_t speed = servoClampHz(hz);
        acServo->setSpeedInHz(speed > 0 ? speed : 1000);
        acServo->setAcceleration(servoAccel);
        acServo->move(pulses);
        servoMode = SERVO_MOVING;
        servoWatchdogTrip = false;
        Serial.printf("AC servo: MOVE %ld pulses at %lu Hz\n", (long)pulses, (unsigned long)speed);
    }
    return true;
}

static void acServoSendStatus() {
    bool running = acServo->isRunning();
    float actualHz = acServo->getCurrentSpeedInMilliHz() / 1000.0f;
    uint8_t state;
    if (servoFault) state = SERVO_FAULT;
    else if (!running) state = SERVO_IDLE;
    else if (servoMode == SERVO_RUNNING) state = SERVO_RUNNING;
    else if (servoMode == SERVO_MOVING) state = SERVO_MOVING;
    else state = SERVO_STOPPING;

    bool atSpeed = running && servoMode == SERVO_RUNNING &&
                   fabsf(fabsf(actualHz) - servoClampHz(servoTargetHz)) <=
                       max(1.0f, 0.01f * servoClampHz(servoTargetHz));
    uint8_t flags = (acServoAlarmActive() ? 0x01 : 0) | (servoWatchdogTrip ? 0x02 : 0) |
                    (atSpeed ? 0x04 : 0) | (SERVO_ALARM_INPUT_ENABLED ? 0x08 : 0);

    CanFrame st = {0};
    st.identifier = CAN_ID_STATUS_SERVO;
    st.extd = 0;
    st.data_length_code = 8;
    st.data[0] = state;
    st.data[1] = flags;
    memcpy(st.data + 4, &actualHz, 4);
    ESP32Can.writeFrame(st);

    CanFrame pos = {0};
    pos.identifier = CAN_ID_STATUS_SERVO_POS;
    pos.extd = 0;
    pos.data_length_code = 8;
    int32_t position = acServo->getCurrentPosition();
    int32_t target = (int32_t)servoTargetHz;
    memcpy(pos.data, &position, 4);
    memcpy(pos.data + 4, &target, 4);
    ESP32Can.writeFrame(pos);
}

void acServoUpdate() {
    if (!acServo) return;
    unsigned long now = millis();

    // Drive alarm -> cut pulses immediately and latch until STOP
    if (acServoAlarmActive() && !servoFault) {
        acServo->forceStop();
        servoMode = SERVO_IDLE;
        servoFault = true;
        Serial.println("AC servo: DRIVE ALARM - pulses stopped");
    }

    // Lost the Pi -> ramp down quickly
    if (acServo->isRunning() && now - servoLastRxMs > SERVO_HEARTBEAT_TIMEOUT_MS) {
        acServoSafetyStop();
        if (!servoWatchdogTrip) Serial.println("AC servo: no CAN heartbeat for 3 s - stopping");
        servoWatchdogTrip = true;
    }

    if (servoMode == SERVO_MOVING && !acServo->isRunning()) servoMode = SERVO_IDLE;

    if (now - servoPrevStatusMs >= SERVO_STATUS_PERIOD_MS) {
        servoPrevStatusMs = now;
        acServoSendStatus();
    }
}
