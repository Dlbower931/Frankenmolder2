#pragma once
/**
 * Frankenmolder - AC servo test control (XP200 drive, pulse + direction mode)
 *
 * Generates step pulses + direction for the servo drive with ramps, exact pulse
 * counts for position moves, and safety stops. Driven by its own CAN IDs, separate
 * from the heater zone commands.
 *
 * REQUIRED LIBRARY: FastAccelStepper (by gin66). Compile-tested with v2.0.1 on ESP32 core 3.3.12.
 *
 * ---------------------------------------------------------------------------
 * WIRING (drive connector CN2, pin numbers from the paper diagram shipped with the drive)
 *   Each signal is an NPN transistor (2N2222A) switching the drive's opto input to ground:
 *     ESP32 GPIO --[1k]--+-- base,  base --[10k]-- GND,  emitter -- GND
 *   ESP32 GND and the 24 V supply's 0 V are tied together.
 *
 *     Signal     ESP32    +24 V to             transistor collector to
 *     pulse      GPIO32   pin 35 (PULLHI 24V)  pin 43 (PULSE-)
 *     direction  GPIO33   pin 35 (same pin)    pin 39 (SIGN-)
 *     SON        GPIO26   pin 11 (COM+)        pin 9  (DI1 / SON)
 *   Leave pin 41 (PULSE+ 5V) and pin 37 (SIGN+ 5V) unconnected. Single-ended 24 V
 *   pulse inputs are good to 200 kHz, the same as SERVO_MAX_HZ.
 *   GPIO HIGH = transistor on = opto on (pulse high / SON on). The 10k keeps SON OFF
 *   while the ESP32 boots or is being flashed.
 *
 *   SON (servo enable) is driven by the firmware: ON when RUN or MOVE is commanded,
 *   pulses start SERVO_SON_SETTLE_MS later (the manual asks for >= 50 ms). OFF once a
 *   STOP has ramped down, immediately on E-STOP, a drive alarm, or when CAN heartbeats
 *   stop. With SON off the motor is free (no holding torque).
 *
 *   Optional alarm input: drive ALM+ (pin 5) to GPIO34 with an external 10k pull-up to
 *   3.3 V (GPIO34 has no internal pull-up), ALM- (pin 4) to ESP32 GND. ALM conducts when
 *   there is NO alarm -> GPIO34 LOW = OK. Set SERVO_ALARM_INPUT_ENABLED to 1 once wired.
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
 *                             bit2 at target speed, bit3 alarm input enabled, bit4 SON on
 *   0x131 SERVO_POS    [i32 position pulses][i32 target_hz echo]
 */

#include <FastAccelStepper.h>

// --- Pins ---
#define SERVO_PULSE_PIN 32
#define SERVO_DIR_PIN   33
#define SERVO_SON_PIN   26            // HIGH = SON on (drive enabled)
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
const unsigned long SERVO_SON_SETTLE_MS = 60;    // SON on -> first pulse (manual: >= 50 ms)

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
// SON + deferred start (pulses wait for SERVO_SON_SETTLE_MS after SON turns on)
static bool servoSonOn = false;
static unsigned long servoSonSinceMs = 0;
static bool servoSonOffWhenStopped = false;  // set by STOP: drop SON once the ramp finishes
enum : uint8_t { PENDING_NONE = 0, PENDING_RUN = 1, PENDING_MOVE = 2 };
static uint8_t servoPending = PENDING_NONE;
static int32_t servoPendingPulses = 0;
static uint32_t servoPendingHz = 0;

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

static void acServoSetSon(bool on) {
    if (on == servoSonOn) return;
    digitalWrite(SERVO_SON_PIN, on ? HIGH : LOW);
    servoSonOn = on;
    if (on) servoSonSinceMs = millis();
    Serial.printf("AC servo: SON %s\n", on ? "ON" : "OFF");
}

static bool acServoSonReady() {
    return servoSonOn && millis() - servoSonSinceMs >= SERVO_SON_SETTLE_MS;
}

// Start or update continuous running at servoTargetHz (ramped).
static void acServoApplyRun() {
    uint32_t hz = servoClampHz(servoTargetHz);
    acServo->setAcceleration(servoAccel);
    if (hz < 1) {                        // target 0 -> ramp to a stop, then SON off
        acServo->stopMove();
        servoPending = PENDING_NONE;
        servoMode = SERVO_IDLE;
        servoSonOffWhenStopped = true;
        return;
    }
    servoMode = SERVO_RUNNING;
    servoSonOffWhenStopped = false;
    if (!acServoSonReady()) {            // enable the drive first, start once it has settled
        acServoSetSon(true);
        servoPending = PENDING_RUN;
        return;
    }
    acServo->setSpeedInHz(hz);
    if (servoTargetHz >= 0) acServo->runForward(); else acServo->runBackward();
}

static void acServoStartMove(int32_t pulses, uint32_t hz) {
    servoMode = SERVO_MOVING;
    servoSonOffWhenStopped = false;
    if (!acServoSonReady()) {
        acServoSetSon(true);
        servoPending = PENDING_MOVE;
        servoPendingPulses = pulses;
        servoPendingHz = hz;
        return;
    }
    acServo->setSpeedInHz(hz > 0 ? hz : 1000);
    acServo->setAcceleration(servoAccel);
    acServo->move(pulses);
}

// Cut pulses now and disable the drive (E-STOP, alarm).
static void acServoKill() {
    acServo->forceStop();
    servoPending = PENDING_NONE;
    servoMode = SERVO_IDLE;
    acServoSetSon(false);
}

static void acServoSafetyStop() {
    acServo->setAcceleration(max(servoAccel, SERVO_SAFETY_DECEL));
    acServo->applySpeedAcceleration();
    acServo->stopMove();
    servoPending = PENDING_NONE;
    servoMode = SERVO_IDLE;
    servoSonOffWhenStopped = true;
}

void acServoSetup() {
    pinMode(SERVO_SON_PIN, OUTPUT);
    digitalWrite(SERVO_SON_PIN, LOW);    // drive disabled until RUN / MOVE
    servoEngine.init();
    acServo = servoEngine.stepperConnectToPin(SERVO_PULSE_PIN);
    if (!acServo) {
        Serial.println("AC servo: FAILED to attach pulse generator to GPIO32");
        return;
    }
    acServo->setDirectionPin(SERVO_DIR_PIN);
    acServo->setAutoEnable(false);       // SON is handled here (acServoSetSon)
    acServo->setSpeedInHz(1000);
    acServo->setAcceleration(servoAccel);
#if SERVO_ALARM_INPUT_ENABLED
    pinMode(SERVO_ALARM_PIN, INPUT);     // external 10k pull-up required
#endif
    servoLastRxMs = millis();
    Serial.printf("AC servo: pulse GPIO%d, dir GPIO%d, SON GPIO%d ready (max %lu Hz)\n",
                  SERVO_PULSE_PIN, SERVO_DIR_PIN, SERVO_SON_PIN, (unsigned long)SERVO_MAX_HZ);
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
                servoPending = PENDING_NONE;
                servoMode = SERVO_IDLE;
                servoSonOffWhenStopped = true;   // SON drops once the ramp has finished
                if (!acServoAlarmActive()) servoFault = false;
                Serial.println("AC servo: STOP (ramp)");
                break;
            case SERVO_CMD_ESTOP:
                acServoKill();
                Serial.println("AC servo: E-STOP (pulses cut, SON off)");
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
        // live speed / direction change (a pending start just picks up the new values)
        if (servoMode == SERVO_RUNNING && servoPending == PENDING_NONE) acServoApplyRun();
        Serial.printf("AC servo: target %.0f Hz, accel %lu Hz/s\n", servoTargetHz, (unsigned long)servoAccel);
    } else if (f.identifier == CAN_ID_CMD_SERVO_MOVE && f.data_length_code >= 8) {
        int32_t pulses;
        float hz;
        memcpy(&pulses, f.data, 4);
        memcpy(&hz, f.data + 4, 4);
        if (servoFault || acServo->isRunning() || servoPending != PENDING_NONE) {
            Serial.println("AC servo: MOVE ignored (running or alarm)");
            return true;
        }
        uint32_t speed = servoClampHz(hz);
        servoWatchdogTrip = false;
        acServoStartMove(pulses, speed);
        Serial.printf("AC servo: MOVE %ld pulses at %lu Hz\n", (long)pulses, (unsigned long)speed);
    }
    return true;
}

static void acServoSendStatus() {
    bool running = acServo->isRunning();
    float actualHz = acServo->getCurrentSpeedInMilliHz() / 1000.0f;
    uint8_t state;
    if (servoFault) state = SERVO_FAULT;
    else if (!running && servoPending == PENDING_NONE) state = SERVO_IDLE;
    else if (servoMode == SERVO_RUNNING) state = SERVO_RUNNING;
    else if (servoMode == SERVO_MOVING) state = SERVO_MOVING;
    else state = SERVO_STOPPING;

    bool atSpeed = running && servoMode == SERVO_RUNNING &&
                   fabsf(fabsf(actualHz) - servoClampHz(servoTargetHz)) <=
                       max(1.0f, 0.01f * servoClampHz(servoTargetHz));
    uint8_t flags = (acServoAlarmActive() ? 0x01 : 0) | (servoWatchdogTrip ? 0x02 : 0) |
                    (atSpeed ? 0x04 : 0) | (SERVO_ALARM_INPUT_ENABLED ? 0x08 : 0) |
                    (servoSonOn ? 0x10 : 0);

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
        acServoKill();
        servoFault = true;
        Serial.println("AC servo: DRIVE ALARM - pulses stopped, SON off");
    }

    // Lost the Pi -> ramp down quickly, then SON off
    bool active = acServo->isRunning() || servoPending != PENDING_NONE || servoSonOn;
    if (active && now - servoLastRxMs > SERVO_HEARTBEAT_TIMEOUT_MS) {
        if (!servoWatchdogTrip) Serial.println("AC servo: no CAN heartbeat for 3 s - stopping");
        if (servoPending != PENDING_NONE || acServo->isRunning()) acServoSafetyStop();
        servoSonOffWhenStopped = true;
        servoWatchdogTrip = true;
    }

    // Deferred start once SON has settled
    if (servoPending != PENDING_NONE && acServoSonReady()) {
        uint8_t p = servoPending;
        servoPending = PENDING_NONE;
        if (p == PENDING_RUN) acServoApplyRun();
        else acServoStartMove(servoPendingPulses, servoPendingHz);
    }

    if (servoMode == SERVO_MOVING && servoPending == PENDING_NONE && !acServo->isRunning())
        servoMode = SERVO_IDLE;   // move finished - SON stays on and holds position

    // STOP / watchdog: drop SON once the motor has stopped
    if (servoSonOffWhenStopped && servoSonOn && !acServo->isRunning() &&
        servoPending == PENDING_NONE) {
        acServoSetSon(false);
        servoSonOffWhenStopped = false;
    }

    if (now - servoPrevStatusMs >= SERVO_STATUS_PERIOD_MS) {
        servoPrevStatusMs = now;
        acServoSendStatus();
    }
}
