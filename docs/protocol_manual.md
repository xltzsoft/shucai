# Serial Communication Protocol Manual

# ESP32-C3 Sensor Hub (ShuCai)

**Baud Rate**: 921600  |  **Data Bits**: 8  |  **Stop Bits**: 1  |  **Parity**: None

---

## 1. Data Frame (ESP32 → Host PC)

Sent at **1kHz** (every 1ms). Total **43 bytes**.

### Frame Structure

| Offset | Len | Field | Value / Description |
|--------|-----|-------|---------------------|
| 0 | 1 | Header byte 0 | `0xAA` |
| 1 | 1 | Header byte 1 | `0x55` |
| 2 | 1 | Payload length | `0x29` (41) |
| 3 | 1 | Frame type | `0x01` = Data |
| 4-7 | 4 | Sequence number | uint32 LE, auto-increment from 0 |
| 8-9 | 2 | Angle (raw) | uint16 LE, AS5048A 14-bit (0~16383) |
| 10-13 | 4 | Sensor1 CH0 | int32 LE, pressure value |
| 14-17 | 4 | Sensor1 CH1 | int32 LE, pressure value |
| 18-21 | 4 | Sensor1 CH2 | int32 LE, pressure value |
| 22-25 | 4 | Sensor1 CH3 | int32 LE, pressure value |
| 26-29 | 4 | Sensor2 CH0 | int32 LE, pressure value |
| 30-33 | 4 | Sensor2 CH1 | int32 LE, pressure value |
| 34-37 | 4 | Sensor2 CH2 | int32 LE, pressure value |
| 38-41 | 4 | Sensor2 CH3 | int32 LE, pressure value |
| 42 | 1 | Checksum | XOR of bytes [2..41] |

### Angle Conversion

```
angle_degrees = raw_angle * 360.0 / 16384.0
```

### Hex Example

```
AA 55 29 01 00 00 00 00 FF 0F  [header + seq=0 + angle=4095]
E8 03 00 00  [S1_CH0 = 1000]
D0 07 00 00  [S1_CH1 = 2000]
B8 0B 00 00  [S1_CH2 = 3000]
A0 0F 00 00  [S1_CH3 = 4000]
88 13 00 00  [S2_CH0 = 5000]
70 17 00 00  [S2_CH1 = 6000]
58 1B 00 00  [S2_CH2 = 7000]
40 1F 00 00  [S2_CH3 = 8000]
XX           [checksum]
```

---

## 2. Command Frame (Host PC → ESP32)

Total **6 bytes**.

### Frame Structure

| Offset | Len | Field | Value / Description |
|--------|-----|-------|---------------------|
| 0 | 1 | Header byte 0 | `0xAA` |
| 1 | 1 | Header byte 1 | `0x55` |
| 2 | 1 | Payload length | `0x04` |
| 3 | 1 | Command type | See command table |
| 4 | 1 | Parameter | See command table |
| 5 | 1 | Checksum | XOR of bytes [2..4] |

### Command Table

| Cmd (Hex) | Name | Param | Description |
|-----------|------|-------|-------------|
| `0x10` | Calibrate Sensor 1 | `0x00` | Zero calibration for I2C_0 sensor |
| `0x11` | Calibrate Sensor 2 | `0x00` | Zero calibration for I2C_1 sensor |
| `0x12` | Calibrate All | `0x00` | Zero calibration for both sensors |
| `0x20` | Start/Stop Stream | `0x01`=start, `0x00`=stop | Enable or disable data output |

### Command Examples

**Calibrate Sensor 1:**

```
AA 55 04 10 00 14
```

Checksum: `0x04 ^ 0x10 ^ 0x00 = 0x14`

**Calibrate All Sensors:**

```
AA 55 04 12 00 16
```

**Stop Data Stream:**

```
AA 55 04 20 00 24
```

**Start Data Stream:**

```
AA 55 04 20 01 25
```

---

## 3. Parsing Guide (Python Example)

```python
import struct
import serial

ser = serial.Serial('COM3', 921600, timeout=0.01)

def parse_frame(data):
    """Parse a 43-byte data frame"""
    if len(data) < 43:
        return None
    if data[0] != 0xAA or data[1] != 0x55:
        return None

    # Verify checksum
    xor_check = 0
    for b in data[2:42]:
        xor_check ^= b
    if xor_check != data[42]:
        return None

    # Parse fields
    payload_len = data[2]
    frame_type  = data[3]
    seq         = struct.unpack_from('<I', data, 4)[0]
    raw_angle   = struct.unpack_from('<H', data, 8)[0]
    angle_deg   = raw_angle * 360.0 / 16384.0

    # 8 channels of int32 pressure
    pressures = []
    for i in range(8):
        p = struct.unpack_from('<i', data, 10 + i * 4)[0]
        pressures.append(p)

    return {
        'seq': seq,
        'angle': angle_deg,
        'sensor1': pressures[0:4],
        'sensor2': pressures[4:8],
    }

# Read loop
buf = bytearray()
while True:
    buf += ser.read(256)
    # Search for frame header
    while len(buf) >= 43:
        idx = buf.find(b'\xAA\x55')
        if idx < 0:
            buf.clear()
            break
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 43:
            break
        result = parse_frame(buf[:43])
        if result:
            print(f"#{result['seq']} angle={result['angle']:.2f} "
                  f"S1={result['sensor1']} S2={result['sensor2']}")
        buf = buf[43:]
```

---

## 4. Wiring Diagram

```
ESP32-C3 (LuatOS C3)            External Devices
============================    ======================
  GPIO6  ──────────────────── AS5048A CLK
  GPIO7  ──────────────────── AS5048A MOSI
  GPIO2  ──────────────────── AS5048A MISO
  GPIO10 ──────────────────── AS5048A CSn
  3.3V   ──────────────────── AS5048A VDD
  GND    ──────────────────── AS5048A GND

  GPIO4  ──────────────────── Sensor1 SDA (I2C_0)
  GPIO5  ──────────────────── Sensor1 SCL (I2C_0)

  GPIO8  ──────────────────── Sensor2 SDA (I2C_1)
  GPIO9  ──────────────────── Sensor2 SCL (I2C_1)

  GPIO21 ──────────────────── USB-TTL RX  (Serial TX)
  GPIO20 ──────────────────── USB-TTL TX  (Serial RX)
  GND    ──────────────────── USB-TTL GND
```
