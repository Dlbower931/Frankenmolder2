#pragma once
/**
 * Frankenmolder - XP200 servo drive monitoring + parameter access over RS-485 (Modbus RTU)
 *
 * The ESP32 is the Modbus master, the drive is slave 1. Non-blocking: one request is in
 * flight at a time and acServoModbusUpdate() (called from loop()) advances it.
 * Pulses still come from ac_servo.h; this link only reads drive state and parameters,
 * plus it latches a servo fault when the drive reports an alarm code (the ALM wire is
 * not needed for that).
 *
 * ---------------------------------------------------------------------------
 * WIRING  (auto-direction TTL<->RS-485 module: TXD, RXD, VCC, GND | GND, A, B)
 *   Power the module from the ESP32's 3.3 V so its TXD output never exceeds 3.3 V
 *   (ESP32 GPIO is NOT 5 V tolerant).
 *
 *     Module   ESP32
 *     VCC      3.3 V
 *     GND      GND
 *     RXD      GPIO22   (UART2 TX -> data into the module)
 *     TXD      GPIO35   (UART2 RX <- data out of the module; input-only pin is fine)
 *   Some boards label TXD/RXD from the other side: if the drive never answers, swap these two.
 *
 *   Alternative: a MAX485-style module with DI/RO/DE/RE -> DI=GPIO22, RO=GPIO35,
 *   DE+RE tied to GPIO21, and set RS485_DE_PIN to 21.
 *
 *     Module   Drive CN3 (RJ45, XP200-20AL/30AL; T568B colours)
 *     A        pins 3 + 4  (485_A)  white/green + blue
 *     B        pins 5 + 6  (485_B)  white/blue  + green
 *     GND      pins 7 + 8  (485 ground) white/brown + brown  -> also ESP32 GND
 *   If nothing answers, swap A and B (vendors disagree on the naming).
 *   Keep a 120 ohm terminator across A/B at the module end (most boards have one fitted).
 *
 * AUTO SCAN: if the drive has not answered for 3 s, the ESP32 tries every baud rate /
 * parity / stop-bit setting the drive supports at station numbers 1-4, and prints
 * "Modbus: FOUND ..." on the serial monitor when one answers (then keeps using it).
 *
 * DRIVE PARAMETERS (set on the drive keypad, save, then power-cycle the drive):
 *   Pr181 = 1   station number (default -1 = RS-485 OFF)
 *   Pr182 = 2   19200 baud (default)
 *   Pr183 = 1   8 data bits, even parity, 1 stop (default)
 *
 * ---------------------------------------------------------------------------
 * CAN PROTOCOL (little-endian, standard IDs)
 *   ESP32 -> Pi (every 100 ms)
 *   0x132 SERVO_DRIVE1 [i16 motor rpm][i16 torque % of rated][i16 current 0.1 A][u16 alarm code]
 *   0x133 SERVO_DRIVE2 [i32 position deviation pulses][u8 avg load %][u8 flags][u16 errors]
 *                      flags: bit0 Modbus link OK, bit1 drive alarm (alarm code != 0)
 *                      errors: Modbus timeouts + CRC errors since boot (wraps)
 *   Pi -> ESP32
 *   0x233 SERVO_PARAM  [u8 op][u8 0][u16 param no][i16 value]   op 1=READ 2=WRITE 3=SAVE to EEPROM
 *   ESP32 -> Pi
 *   0x134 SERVO_PARAM_REPLY [u8 op][u8 status][u16 param no][i16 value]
 *                      status 0=OK 1=timeout 2=bad CRC/frame 3=busy (queue full) 4=no link setup,
 *                      0x80|code = Modbus exception from the drive
 */

#include "hal/uart_types.h"

// --- Master switch ---
// 0 = RS-485 link not used (shelved 2026-11-11: the drive never answered). Nothing is sent,
//     no 0x132/0x133 status frames, and drive alarms come only from the ALM input.
#define SERVO_MODBUS_ENABLED 0

// --- Pins / serial ---
#define RS485_TX_PIN 22
#define RS485_RX_PIN 35
#define RS485_DE_PIN -1          // auto-direction module; 21 for a MAX485 with DE/RE
#define RS485_BAUD   19200       // Pr182 = 2
#define RS485_CONFIG SERIAL_8E1  // Pr183 = 1
#define SERVO_MODBUS_ADDR 1      // Pr181
#define MB_SCAN_ENABLED 1        // search baud/parity/address when the drive does not answer
static HardwareSerial& rs485 = Serial2;

// --- CAN IDs ---
#define CAN_ID_STATUS_SERVO_DRIVE1 0x132
#define CAN_ID_STATUS_SERVO_DRIVE2 0x133
#define CAN_ID_STATUS_SERVO_PARAM  0x134
#define CAN_ID_CMD_SERVO_PARAM     0x233

enum : uint8_t { MB_OP_READ = 1, MB_OP_WRITE = 2, MB_OP_SAVE = 3 };
enum : uint8_t { MB_OK = 0, MB_TIMEOUT = 1, MB_BAD_FRAME = 2, MB_BUSY = 3, MB_NO_SETUP = 4 };

// --- Timing ---
const unsigned long MB_RESPONSE_TIMEOUT_MS = 60;
const unsigned long MB_SAVE_TIMEOUT_MS = 1000;     // EEPROM save answers slowly
const unsigned long MB_POLL_GAP_MS = 25;           // idle time between requests (>> 3.5 chars)
const unsigned long MB_LINK_TIMEOUT_MS = 1000;     // link OK if a good reply within this
const unsigned long MB_STATUS_PERIOD_MS = 100;
const unsigned long MB_DEBUG_PERIOD_MS = 2000;     // serial monitor summary
const unsigned long MB_SCAN_AFTER_MS = 3000;       // no reply this long -> start scanning
const unsigned long MB_SCAN_TIMEOUT_MS = 120;      // per scan attempt (covers 4800 baud)

// Serial settings in use (start at the defines; the scan may change them)
static uint32_t mbBaud = RS485_BAUD;
static uint32_t mbConfig = RS485_CONFIG;
static uint8_t mbAddr = SERVO_MODBUS_ADDR;

// Scan candidates: every setting Pr182 / Pr183 allow, station numbers 1..MB_SCAN_MAX_ADDR
static const uint32_t MB_SCAN_BAUDS[] = {19200, 9600, 38400, 4800};
static const uint32_t MB_SCAN_CONFIGS[] = {SERIAL_8E1, SERIAL_8N1, SERIAL_8O1,
                                           SERIAL_8N2, SERIAL_8E2, SERIAL_8O2};
static const char* const MB_SCAN_CONFIG_NAMES[] = {"8E1 (Pr183=1)", "8N1 (Pr183=0)",
                                                   "8O1 (Pr183=2)", "8N2 (Pr183=3)",
                                                   "8E2 (Pr183=4)", "8O2 (Pr183=5)"};
const uint8_t MB_SCAN_MAX_ADDR = 4;
const uint16_t MB_SCAN_COUNT = 4 * 6 * MB_SCAN_MAX_ADDR;
static bool mbScanning = false;
static uint16_t mbScanIdx = 0;
static uint32_t mbScanSweeps = 0;
static unsigned long mbNoReplySinceMs = 0;

// --- Drive state (from function 04 status registers) ---
struct ServoDriveState {
    int16_t rpm = 0;            // 0x0000 motor speed r/min
    int32_t deviation = 0;      // 0x0007/0x0008 position deviation, pulses
    int16_t torque = 0;         // 0x0009 % of rated
    int16_t current = 0;        // 0x000B 0.1 A
    uint16_t load = 0;          // 0x0018 average load rate %
    uint16_t alarm = 0;         // 0x001A alarm code (0 = none)
};
static ServoDriveState driveState;
static bool mbReady = false;
static unsigned long mbLastGoodMs = 0;
static uint16_t mbErrors = 0;
static unsigned long mbPrevStatusMs = 0;
static unsigned long mbPrevDebugMs = 0;
static uint32_t mbGoodReplies = 0, mbTimeouts = 0, mbBadFrames = 0, mbRxBytes = 0;
static uint32_t mbEchoBytes = 0;          // bytes seen while we were transmitting
static uint8_t mbLastRaw[12];             // last bytes received, for the serial monitor
static uint8_t mbLastRawLen = 0;

// Polled status blocks (function 04, max 8 registers per read)
struct MbPoll { uint16_t start; uint8_t count; };
static const MbPoll MB_POLLS[] = {
    {0x0000, 1},   // speed
    {0x0007, 5},   // deviation lo/hi, torque, peak torque, current
    {0x0018, 3},   // avg load, output voltage, alarm code
};
static uint8_t mbPollIdx = 0;

// One queued parameter request from the Pi (handled before the next poll)
static bool mbParamPending = false;
static uint8_t mbParamOp = 0;
static uint16_t mbParamNo = 0;
static int16_t mbParamValue = 0;

// Transaction in flight
enum : uint8_t { TX_NONE = 0, TX_POLL = 1, TX_PARAM = 2 };
static uint8_t mbTxKind = TX_NONE;
static uint8_t mbExpectFunc = 0;
static uint8_t mbExpectLen = 0;
static unsigned long mbTxMs = 0;
static unsigned long mbTimeoutMs = MB_RESPONSE_TIMEOUT_MS;
static unsigned long mbIdleSinceMs = 0;
static uint8_t mbRx[32];
static uint8_t mbRxLen = 0;

static uint16_t mbCrc(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

inline bool servoModbusLinkOk() {
    return mbReady && mbLastGoodMs != 0 && millis() - mbLastGoodMs < MB_LINK_TIMEOUT_MS;
}

// Drive reports an alarm (only trusted while the link is up)
inline bool servoModbusAlarm() { return servoModbusLinkOk() && driveState.alarm != 0; }

static void mbSend(uint8_t* frame, uint8_t len, uint8_t kind, uint8_t expectLen,
                   unsigned long timeoutMs) {
    uint16_t crc = mbCrc(frame, len);
    frame[len++] = crc & 0xFF;          // CRC low byte first
    frame[len++] = crc >> 8;
    while (rs485.available()) { rs485.read(); mbRxBytes++; }   // stray bytes
    rs485.write(frame, len);
    rs485.flush(true);                  // wait until sent; RTS/DE drops after the stop bit
    while (rs485.available()) { rs485.read(); mbEchoBytes++; }  // our own echo, if any
    mbTxKind = kind;
    mbExpectFunc = frame[1];
    mbExpectLen = expectLen;
    mbRxLen = 0;
    mbTxMs = millis();
    mbTimeoutMs = timeoutMs;
}

static void mbSendRead(uint8_t func, uint16_t start, uint8_t count, uint8_t kind) {
    uint8_t f[8] = {mbAddr, func, (uint8_t)(start >> 8), (uint8_t)start, 0, count};
    mbSend(f, 6, kind, 5 + 2 * count, MB_RESPONSE_TIMEOUT_MS);
}

static void mbSendParamRequest() {
    if (mbParamOp == MB_OP_READ) {
        mbSendRead(0x03, mbParamNo, 1, TX_PARAM);
    } else if (mbParamOp == MB_OP_WRITE) {
        uint8_t f[8] = {mbAddr, 0x06, (uint8_t)(mbParamNo >> 8), (uint8_t)mbParamNo,
                        (uint8_t)((uint16_t)mbParamValue >> 8), (uint8_t)mbParamValue};
        mbSend(f, 6, TX_PARAM, 8, MB_RESPONSE_TIMEOUT_MS);
    } else {
        uint8_t f[4] = {mbAddr, 0x41};
        mbSend(f, 2, TX_PARAM, 4, MB_SAVE_TIMEOUT_MS);
    }
}

static void mbParamReply(uint8_t status, int16_t value) {
    CanFrame r = {0};
    r.identifier = CAN_ID_STATUS_SERVO_PARAM;
    r.extd = 0;
    r.data_length_code = 6;
    r.data[0] = mbParamOp;
    r.data[1] = status;
    memcpy(r.data + 2, &mbParamNo, 2);
    memcpy(r.data + 4, &value, 2);
    ESP32Can.writeFrame(r);
    Serial.printf("Modbus: param op %u Pr%03u -> status 0x%02X value %d\n", mbParamOp, mbParamNo,
                  status, value);
}

static inline uint16_t mbReg(int i) { return ((uint16_t)mbRx[3 + 2 * i] << 8) | mbRx[4 + 2 * i]; }

// A complete, CRC-checked reply for the request in flight
static void mbHandleReply() {
    mbLastGoodMs = millis();
    mbGoodReplies++;
    if (mbScanning) {
        mbScanning = false;
        Serial.printf("Modbus: FOUND drive at %lu baud, %s, station %u - using it. Set "
                      "RS485_BAUD/RS485_CONFIG/SERVO_MODBUS_ADDR to match (or change the drive).\n",
                      (unsigned long)mbBaud, MB_SCAN_CONFIG_NAMES[(mbScanIdx / MB_SCAN_MAX_ADDR) % 6],
                      mbAddr);
        return;   // the scan's speed read is not part of the poll cycle
    }
    if (mbTxKind == TX_PARAM) {
        int16_t value = mbParamValue;
        if (mbParamOp == MB_OP_READ) value = (int16_t)mbReg(0);
        mbParamPending = false;
        mbParamReply(MB_OK, value);
        return;
    }
    const MbPoll& p = MB_POLLS[mbPollIdx];
    if (p.start == 0x0000) {
        driveState.rpm = (int16_t)mbReg(0);
    } else if (p.start == 0x0007) {
        driveState.deviation = (int32_t)(((uint32_t)mbReg(1) << 16) | mbReg(0));
        driveState.torque = (int16_t)mbReg(2);
        driveState.current = (int16_t)mbReg(4);
    } else if (p.start == 0x0018) {
        driveState.load = mbReg(0);
        uint16_t alarm = mbReg(2);
        if (alarm != driveState.alarm) Serial.printf("Modbus: drive alarm code %u\n", alarm);
        driveState.alarm = alarm;
    }
    mbPollIdx = (mbPollIdx + 1) % (sizeof(MB_POLLS) / sizeof(MB_POLLS[0]));
}

static void mbFinishError(uint8_t status) {
    if (status == MB_TIMEOUT || status == MB_BAD_FRAME) mbErrors++;
    if (status == MB_TIMEOUT) mbTimeouts++;
    if (status == MB_BAD_FRAME) mbBadFrames++;
    if (mbTxKind == TX_PARAM) {
        mbParamPending = false;
        mbParamReply(status, 0);
    } else {
        mbPollIdx = (mbPollIdx + 1) % (sizeof(MB_POLLS) / sizeof(MB_POLLS[0]));
    }
}

static void mbSendStatus() {
    bool link = servoModbusLinkOk();
    CanFrame a = {0};
    a.identifier = CAN_ID_STATUS_SERVO_DRIVE1;
    a.extd = 0;
    a.data_length_code = 8;
    memcpy(a.data + 0, &driveState.rpm, 2);
    memcpy(a.data + 2, &driveState.torque, 2);
    memcpy(a.data + 4, &driveState.current, 2);
    memcpy(a.data + 6, &driveState.alarm, 2);
    ESP32Can.writeFrame(a);

    CanFrame b = {0};
    b.identifier = CAN_ID_STATUS_SERVO_DRIVE2;
    b.extd = 0;
    b.data_length_code = 8;
    memcpy(b.data, &driveState.deviation, 4);
    b.data[4] = (uint8_t)min<uint16_t>(driveState.load, 255);
    b.data[5] = (link ? 0x01 : 0) | (servoModbusAlarm() ? 0x02 : 0);
    memcpy(b.data + 6, &mbErrors, 2);
    ESP32Can.writeFrame(b);
}

static bool mbOpenSerial() {
    rs485.end();
    rs485.begin(mbBaud, mbConfig, RS485_RX_PIN, RS485_TX_PIN);
    if (RS485_DE_PIN >= 0) {
        if (!rs485.setPins(-1, -1, -1, RS485_DE_PIN) ||
            !rs485.setMode(UART_MODE_RS485_HALF_DUPLEX)) {
            Serial.println("Modbus: FAILED to set RS-485 half-duplex mode");
            return false;
        }
    }
    return true;
}

// Point the serial port / address at scan candidate mbScanIdx
static void mbApplyScanCandidate() {
    uint16_t i = mbScanIdx;
    uint8_t addr = 1 + i % MB_SCAN_MAX_ADDR;
    i /= MB_SCAN_MAX_ADDR;
    uint32_t config = MB_SCAN_CONFIGS[i % 6];
    uint32_t baud = MB_SCAN_BAUDS[(i / 6) % 4];
    mbAddr = addr;
    if (baud != mbBaud || config != mbConfig) {
        mbBaud = baud;
        mbConfig = config;
        mbOpenSerial();
    }
}

void servoModbusSetup() {
    if (!SERVO_MODBUS_ENABLED) {
        Serial.println("Modbus: RS-485 link disabled (SERVO_MODBUS_ENABLED 0)");
        return;
    }
    if (!mbOpenSerial()) return;
    mbReady = true;
    mbIdleSinceMs = mbNoReplySinceMs = millis();
    Serial.printf("Modbus: RS-485 TX GPIO%d RX GPIO%d DE GPIO%d, %d baud 8E1, slave %d\n",
                  RS485_TX_PIN, RS485_RX_PIN, RS485_DE_PIN, RS485_BAUD, SERVO_MODBUS_ADDR);
}

// Returns true if the frame was a Modbus parameter command.
bool servoModbusHandleFrame(const CanFrame& f) {
    if (f.identifier != CAN_ID_CMD_SERVO_PARAM) return false;
    if (f.data_length_code < 6) return true;
    uint8_t op = f.data[0];
    uint16_t no;
    int16_t value;
    memcpy(&no, f.data + 2, 2);
    memcpy(&value, f.data + 4, 2);
    if (op < MB_OP_READ || op > MB_OP_SAVE) return true;
    if (!mbReady || mbParamPending) {
        uint8_t saveOp = mbParamOp; uint16_t saveNo = mbParamNo;   // keep the queued request
        mbParamOp = op; mbParamNo = no;
        mbParamReply(mbReady ? MB_BUSY : MB_NO_SETUP, 0);
        mbParamOp = saveOp; mbParamNo = saveNo;
        return true;
    }
    mbParamOp = op;
    mbParamNo = no;
    mbParamValue = value;
    mbParamPending = true;
    return true;
}

void servoModbusUpdate() {
    if (!mbReady) return;
    unsigned long now = millis();

    if (mbTxKind != TX_NONE) {
        while (rs485.available() && mbRxLen < sizeof(mbRx)) {
            uint8_t c = rs485.read();
            mbRx[mbRxLen++] = c;
            mbRxBytes++;
            if (mbLastRawLen < sizeof(mbLastRaw)) mbLastRaw[mbLastRawLen++] = c;
        }
        uint8_t status = 0xFF;   // 0xFF = still waiting
        if (mbRxLen >= 5 && mbRx[1] == (mbExpectFunc | 0x80)) {        // exception reply
            status = (mbCrc(mbRx, 5) == 0 && mbRx[0] == mbAddr) ? (0x80 | mbRx[2])
                                                                            : MB_BAD_FRAME;
            if (status & 0x80) mbLastGoodMs = now;   // the drive answered
        } else if (mbRxLen >= mbExpectLen) {
            bool ok = mbRx[0] == mbAddr && mbRx[1] == mbExpectFunc &&
                      mbCrc(mbRx, mbExpectLen) == 0;   // CRC over data+CRC is 0 when valid
            if (ok) {
                mbHandleReply();
                mbTxKind = TX_NONE;
                mbIdleSinceMs = now;
            } else {
                status = MB_BAD_FRAME;
            }
        } else if (now - mbTxMs > mbTimeoutMs) {
            status = MB_TIMEOUT;
        }
        if (status != 0xFF) {
            if (mbScanning && (status & 0x80)) {   // an exception reply still means it answered
                mbHandleReply();
            } else if (mbScanning) {
                if (mbRxLen) Serial.printf("Modbus scan: %u bytes (not a valid reply) at %lu baud %s "
                                           "station %u\n", mbRxLen, (unsigned long)mbBaud,
                                           MB_SCAN_CONFIG_NAMES[(mbScanIdx / MB_SCAN_MAX_ADDR) % 6],
                                           mbAddr);
                if (++mbScanIdx >= MB_SCAN_COUNT) {
                    mbScanIdx = 0;
                    mbScanSweeps++;
                    Serial.printf("Modbus scan: sweep %lu done - no answer at any baud/parity, "
                                  "station 1-%u\n", (unsigned long)mbScanSweeps, MB_SCAN_MAX_ADDR);
                }
                mbApplyScanCandidate();
            } else {
                mbFinishError(status);
            }
            mbTxKind = TX_NONE;
            mbIdleSinceMs = now;
        }
    } else if (now - mbIdleSinceMs >= MB_POLL_GAP_MS) {
        if (servoModbusLinkOk()) mbNoReplySinceMs = now;
#if MB_SCAN_ENABLED
        if (!mbScanning && now - mbNoReplySinceMs > MB_SCAN_AFTER_MS) {
            mbScanning = true;
            mbScanIdx = 0;
            Serial.println("Modbus: no reply for 3 s - scanning baud rates / parity / station 1-4");
            mbApplyScanCandidate();
        }
#endif
        if (mbScanning) {
            if (mbParamPending) { mbParamPending = false; mbParamReply(MB_TIMEOUT, 0); }
            uint8_t f[8] = {mbAddr, 0x04, 0, 0, 0, 1};   // read motor speed
            mbSend(f, 6, TX_POLL, 7, MB_SCAN_TIMEOUT_MS);
        } else if (mbParamPending) mbSendParamRequest();
        else {
            const MbPoll& p = MB_POLLS[mbPollIdx];
            mbSendRead(0x04, p.start, p.count, TX_POLL);
        }
    }

    if (now - mbPrevStatusMs >= MB_STATUS_PERIOD_MS) {
        mbPrevStatusMs = now;
        mbSendStatus();
    }

    // Serial monitor summary (counts are since boot)
    if (now - mbPrevDebugMs >= MB_DEBUG_PERIOD_MS) {
        mbPrevDebugMs = now;
        if (servoModbusLinkOk()) {
            Serial.printf("Modbus: link OK | %d rpm, torque %d%%, %.1f A, dev %ld pls, load %u%%, "
                          "alarm %u | good %lu, timeouts %lu, bad %lu\n",
                          driveState.rpm, driveState.torque, driveState.current / 10.0f,
                          (long)driveState.deviation, driveState.load, driveState.alarm,
                          (unsigned long)mbGoodReplies, (unsigned long)mbTimeouts,
                          (unsigned long)mbBadFrames);
        } else {
            Serial.printf("Modbus: NO REPLY from drive%s | bytes received %lu, echo bytes %lu, "
                          "timeouts %lu, bad frames %lu", mbScanning ? " (scanning)" : "",
                          (unsigned long)mbRxBytes, (unsigned long)mbEchoBytes,
                          (unsigned long)mbTimeouts, (unsigned long)mbBadFrames);
            if (mbLastRawLen) {
                Serial.print(" | first bytes:");
                for (uint8_t i = 0; i < mbLastRawLen; i++) Serial.printf(" %02X", mbLastRaw[i]);
            }
            Serial.println();
        }
    }
}
