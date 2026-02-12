#include <Arduino.h>
#include <Wire.h>
#include "AS5048A.h"

// Forward declarations
void handleCommand(uint8_t cmd, uint8_t param);
bool switchI2CPins(int sda, int scl);

// ============================================================
// Pin Definitions
// ============================================================

// --- SPI Pins (AS5048A) ---
#define PIN_SPI_SCK   6
#define PIN_SPI_MISO  2
#define PIN_SPI_MOSI  7
#define PIN_AS5048_CS 10

// --- I2C Sensor 1 Pins ---
#define PIN_I2C0_SDA  4
#define PIN_I2C0_SCL  5

// --- I2C Sensor 2 Pins ---
#define PIN_I2C1_SDA  8
#define PIN_I2C1_SCL  9

// ============================================================
// I2C Sensor Protocol Constants
// ============================================================
#define SENSOR_I2C_ADDR       0x2C   // 7-bit slave address
#define SENSOR_CMD_READ_HI    0x55
#define SENSOR_CMD_READ_LO    0xAA
#define SENSOR_CMD_CALI_LO    0xAB
#define SENSOR_RESPONSE_LEN   42     // bytes returned by slave
#define SENSOR_CHANNELS       4      // pressure channels per sensor
#define I2C_CLOCK_HZ          400000 // 400kHz Fast Mode

// ============================================================
// Serial Protocol Constants (ESP32 <-> Host PC)
// ============================================================
#define FRAME_HEADER_0        0xAA
#define FRAME_HEADER_1        0x55
#define FRAME_TYPE_DATA       0x01
#define FRAME_TYPE_ACK        0x02

// Command types from host
#define CMD_CALI_SENSOR1      0x10
#define CMD_CALI_SENSOR2      0x11
#define CMD_CALI_ALL          0x12
#define CMD_START_STOP        0x20

// Data frame: 2(header) + 1(len) + 1(type) + 4(seq) + 2(angle) + 16(s1) + 16(s2) + 1(xor) = 43
#define DATA_FRAME_PAYLOAD_LEN  41
#define DATA_FRAME_TOTAL_LEN    43

// ============================================================
// Global Objects & Variables
// ============================================================

AS5048A encoder(PIN_AS5048_CS);

// Sensor raw response buffers
static uint8_t sensorBuf1[SENSOR_RESPONSE_LEN];
static uint8_t sensorBuf2[SENSOR_RESPONSE_LEN];

// Parsed pressure values (4 channels per sensor)
static int32_t pressure1[SENSOR_CHANNELS];
static int32_t pressure2[SENSOR_CHANNELS];

// Track which I2C pins are currently active
static int currentSDA = -1;
static int currentSCL = -1;

// Frame sequence number
static uint32_t frameSeq = 0;

// Data streaming control
static bool streamingEnabled = true;

// Serial TX frame buffer
static uint8_t txFrame[DATA_FRAME_TOTAL_LEN];

// Serial RX command buffer
static uint8_t rxBuf[8];
static uint8_t rxIdx = 0;

// Debug mode: print I2C errors for first N frames
static uint32_t debugCount = 0;
#define DEBUG_FRAMES  200  // Print debug info for first 200 frames

// Timing
static unsigned long lastSampleTime = 0;
#define SAMPLE_INTERVAL_US  15000  // 15ms (~66Hz) to accommodate I2C delays

// ============================================================
// I2C Pin Switching (ESP32-C3 has only 1 I2C controller)
// ============================================================

/**
 * @brief Switch I2C Wire to different pins
 *        ESP32-C3 only has 1 I2C peripheral, so we share it
 *        between two sensor connections by re-initializing with
 *        different pins.
 * @return true if initialization succeeded
 */
bool switchI2CPins(int sda, int scl)
{
    if (currentSDA == sda && currentSCL == scl) {
        return true;  // Already on these pins
    }

    // Must end current session before switching pins
    if (currentSDA >= 0) {
        Wire.end();
    }

    bool ok = Wire.begin(sda, scl);
    if (ok) {
        Wire.setClock(I2C_CLOCK_HZ);
        Wire.setTimeOut(10);  // 10ms timeout, give slave time to respond
        currentSDA = sda;
        currentSCL = scl;
    } else {
        currentSDA = -1;
        currentSCL = -1;
    }
    return ok;
}

// ============================================================
// I2C Sensor Read Functions
// ============================================================

/**
 * @brief Send 0x55AA command and read response from I2C sensor
 * @param sda  SDA pin for this sensor
 * @param scl  SCL pin for this sensor
 * @param buf  Buffer to store response (SENSOR_RESPONSE_LEN bytes)
 * @return true if read succeeded
 */
bool readSensor(int sda, int scl, uint8_t *buf)
{
    // Switch I2C to correct pins
    if (!switchI2CPins(sda, scl)) {
        if (debugCount < DEBUG_FRAMES) {
            Serial.printf("[I2C] switchPins(%d,%d) FAIL\n", sda, scl);
        }
        return false;
    }

    // Step 1: Send command 0x55 0xAA
    Wire.beginTransmission(SENSOR_I2C_ADDR);
    Wire.write(SENSOR_CMD_READ_HI);
    Wire.write(SENSOR_CMD_READ_LO);
    uint8_t err = Wire.endTransmission(true);  // send STOP
    if (err != 0) {
        if (debugCount < DEBUG_FRAMES) {
            // err: 1=too long, 2=NACK addr, 3=NACK data, 4=other, 5=timeout
            Serial.printf("[I2C] pins(%d,%d) endTransmission err=%d\n", sda, scl, err);
        }
        return false;
    }

    // Delay for slave to process command and arm TX buffer
    // Slave needs time: interrupt -> PrepareTxFromLatestFrames -> Slave_Transmit_IT
    delay(5);  // 5ms - much more generous than 100us

    // Step 2: Read response
    uint8_t count = Wire.requestFrom((uint8_t)SENSOR_I2C_ADDR, (uint8_t)SENSOR_RESPONSE_LEN);
    if (count < SENSOR_RESPONSE_LEN) {
        if (debugCount < DEBUG_FRAMES) {
            Serial.printf("[I2C] pins(%d,%d) requestFrom got %d/%d bytes\n", sda, scl, count, SENSOR_RESPONSE_LEN);
        }
        return false;
    }

    for (uint8_t i = 0; i < SENSOR_RESPONSE_LEN; i++) {
        buf[i] = Wire.read();
    }

    if (debugCount < DEBUG_FRAMES) {
        Serial.printf("[I2C] pins(%d,%d) OK, hdr=%02X %02X\n", sda, scl, buf[0], buf[1]);
    }
    return true;
}

/**
 * @brief Send calibration (zero) command 0x55AB to sensor
 * @param sda  SDA pin for this sensor
 * @param scl  SCL pin for this sensor
 * @return true if write succeeded
 */
bool calibrateSensor(int sda, int scl)
{
    if (!switchI2CPins(sda, scl)) return false;

    Wire.beginTransmission(SENSOR_I2C_ADDR);
    Wire.write(SENSOR_CMD_READ_HI);
    Wire.write(SENSOR_CMD_CALI_LO);
    return (Wire.endTransmission() == 0);
}

// ============================================================
// Data Parsing
// ============================================================

/**
 * @brief Parse 4-channel int32 pressure values from sensor response
 *
 *   Offset 13..28: 4 channels × int32 LE pressure data
 */
void parsePressure(const uint8_t *buf, int32_t *pressure)
{
    for (int ch = 0; ch < SENSOR_CHANNELS; ch++) {
        uint8_t offset = 13 + ch * 4;
        pressure[ch] = (int32_t)buf[offset]
                     | ((int32_t)buf[offset + 1] << 8)
                     | ((int32_t)buf[offset + 2] << 16)
                     | ((int32_t)buf[offset + 3] << 24);
    }
}

// ============================================================
// Serial Protocol: Build & Send Data Frame
// ============================================================

void sendDataFrame(uint16_t rawAngle, const int32_t *p1, const int32_t *p2)
{
    uint8_t idx = 0;

    // Header
    txFrame[idx++] = FRAME_HEADER_0;
    txFrame[idx++] = FRAME_HEADER_1;

    // Length
    txFrame[idx++] = DATA_FRAME_PAYLOAD_LEN;

    // Frame type
    txFrame[idx++] = FRAME_TYPE_DATA;

    // Sequence number (uint32 LE)
    txFrame[idx++] = (frameSeq >> 0)  & 0xFF;
    txFrame[idx++] = (frameSeq >> 8)  & 0xFF;
    txFrame[idx++] = (frameSeq >> 16) & 0xFF;
    txFrame[idx++] = (frameSeq >> 24) & 0xFF;

    // Angle (uint16 LE)
    txFrame[idx++] = (rawAngle >> 0) & 0xFF;
    txFrame[idx++] = (rawAngle >> 8) & 0xFF;

    // Sensor 1: 4 channels × int32 LE
    for (int ch = 0; ch < SENSOR_CHANNELS; ch++) {
        txFrame[idx++] = (p1[ch] >> 0)  & 0xFF;
        txFrame[idx++] = (p1[ch] >> 8)  & 0xFF;
        txFrame[idx++] = (p1[ch] >> 16) & 0xFF;
        txFrame[idx++] = (p1[ch] >> 24) & 0xFF;
    }

    // Sensor 2: 4 channels × int32 LE
    for (int ch = 0; ch < SENSOR_CHANNELS; ch++) {
        txFrame[idx++] = (p2[ch] >> 0)  & 0xFF;
        txFrame[idx++] = (p2[ch] >> 8)  & 0xFF;
        txFrame[idx++] = (p2[ch] >> 16) & 0xFF;
        txFrame[idx++] = (p2[ch] >> 24) & 0xFF;
    }

    // Checksum: XOR of bytes [2..idx-1]
    uint8_t xorSum = 0;
    for (uint8_t i = 2; i < idx; i++) {
        xorSum ^= txFrame[i];
    }
    txFrame[idx++] = xorSum;

    // Send frame
    Serial.write(txFrame, idx);

    frameSeq++;
}

// ============================================================
// Serial Protocol: Receive & Process Commands
// ============================================================

void processSerialInput()
{
    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (rxIdx == 0) {
            if (b == FRAME_HEADER_0) rxBuf[rxIdx++] = b;
        } else if (rxIdx == 1) {
            if (b == FRAME_HEADER_1) {
                rxBuf[rxIdx++] = b;
            } else {
                rxIdx = 0;
            }
        } else {
            rxBuf[rxIdx++] = b;

            if (rxIdx >= 6) {
                uint8_t len = rxBuf[2];
                if (len == 4) {
                    uint8_t xorCheck = 0;
                    for (uint8_t i = 2; i < 5; i++) {
                        xorCheck ^= rxBuf[i];
                    }
                    if (xorCheck == rxBuf[5]) {
                        handleCommand(rxBuf[3], rxBuf[4]);
                    }
                }
                rxIdx = 0;
            }
        }
    }
}

void handleCommand(uint8_t cmd, uint8_t param)
{
    switch (cmd) {
    case CMD_CALI_SENSOR1:
        calibrateSensor(PIN_I2C0_SDA, PIN_I2C0_SCL);
        break;
    case CMD_CALI_SENSOR2:
        calibrateSensor(PIN_I2C1_SDA, PIN_I2C1_SCL);
        break;
    case CMD_CALI_ALL:
        calibrateSensor(PIN_I2C0_SDA, PIN_I2C0_SCL);
        calibrateSensor(PIN_I2C1_SDA, PIN_I2C1_SCL);
        break;
    case CMD_START_STOP:
        streamingEnabled = (param != 0);
        break;
    default:
        break;
    }
}

// ============================================================
// Setup
// ============================================================
void setup()
{
    // Serial at 921600 for high-speed binary data
    Serial.begin(921600);
    while (!Serial && millis() < 3000);

    // --- Init SPI & AS5048A ---
    encoder.begin(SPI, PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // --- Init I2C on Sensor 1 pins first ---
    // Note: ESP32-C3 has only 1 I2C controller.
    // We share it between two sensors by switching pins.
    switchI2CPins(PIN_I2C0_SDA, PIN_I2C0_SCL);

    // Clear pressure arrays
    memset(pressure1, 0, sizeof(pressure1));
    memset(pressure2, 0, sizeof(pressure2));

    lastSampleTime = micros();
}

// ============================================================
// Loop
// ============================================================
void loop()
{
    // Process any incoming commands from host
    processSerialInput();

    if (!streamingEnabled) return;

    // 1kHz target using micros()
    unsigned long now = micros();
    if (now - lastSampleTime < SAMPLE_INTERVAL_US) return;
    lastSampleTime += SAMPLE_INTERVAL_US;

    // Read AS5048A angle
    uint16_t rawAngle = encoder.getRawAngle();

    // Read I2C Sensor 1 (pins GPIO4/5), zero on failure
    if (readSensor(PIN_I2C0_SDA, PIN_I2C0_SCL, sensorBuf1)) {
        parsePressure(sensorBuf1, pressure1);
    } else {
        memset(pressure1, 0, sizeof(pressure1));
    }

    // Read I2C Sensor 2 (pins GPIO8/9), zero on failure
    if (readSensor(PIN_I2C1_SDA, PIN_I2C1_SCL, sensorBuf2)) {
        parsePressure(sensorBuf2, pressure2);
    } else {
        memset(pressure2, 0, sizeof(pressure2));
    }

    // Build and send binary data frame to host
    sendDataFrame(rawAngle, pressure1, pressure2);

    debugCount++;
}