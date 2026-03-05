# BMI088 C Driver For STM32 Microcontrollers

A C driver for the **Bosch BMI088** 6-axis IMU (accelerometer + gyroscope) over SPI, built for STM32 microcontrollers using the HAL library.

> Author: H. Emirhan Solak · License: MIT

---

Configure both CS pins as **GPIO Output Push-Pull, default HIGH**. Recommended SPI settings: **8-bit, MSB first, Mode 0** (CPOL=0, CPHA=0), ≤ 10 MHz, software NSS.

---

## Integration

1. Copy `bmi088.h` and `bmi088.c` into your project's `Inc/` and `Src/` directories.
2. Update the HAL include at the top of both files to match your target family:
   ```c
   #include "stm32f4xx_hal.h"  // change as needed
   ```
3. Include the header in your application:
   ```c
   #include "bmi088.h"
   ```

---

## Quick Start

```c
BMI088_t imu;

// Init gyroscope (default: ±2000 °/s, normal power mode)
if (BMI088_InitGyro(&imu, &hspi1, GYRO_CS_GPIO_Port, GYRO_CS_Pin) != BMI088_GYRO_OK)
    Error_Handler();

// Init accelerometer (default: ±6g, 100 Hz ODR, normal bandwidth)
if (BMI088_InitAccel(&imu, &hspi1, ACCEL_CS_GPIO_Port, ACCEL_CS_Pin) != BMI088_ACCEL_OK)
    Error_Handler();

// Read in loop
BMI088_FloatData_t gyro, accel;
float tempC;

BMI088_ReadGyroXYZ_DPS(&imu, &gyro);    // °/s
BMI088_ReadAccelXYZ_MG(&imu, &accel);   // g
BMI088_ReadTemp_Celsius(&imu, &tempC);  // °C
```

---

## API Reference

### Gyroscope

| Function | Description |
|---|---|
| `BMI088_InitGyro(sensor, hspi, port, pin)` | Init, verify chip ID (`0x0F`), set defaults |
| `BMI088_ResetGyro(sensor)` | Soft reset, 30 ms wait |
| `BMI088_SetRangeGyro(sensor, range)` | Set range, auto-update sensitivity |
| `BMI088_SetBandwithGyro(sensor, bw)` | Set ODR + filter bandwidth |
| `BMI088_SwitchGyroLPM(sensor, mode)` | `NORMAL` / `SUSPEND` / `DEEP_SUSPEND` |
| `BMI088_SelfTestGyro(sensor)` | Built-in BIST, 100 ms timeout |
| `BMI088_ReadGyroXYZ_LSB(sensor, *raw)` | 3-axis burst, raw int16 |
| `BMI088_ReadGyroXYZ_DPS(sensor, *data)` | 3-axis burst, float °/s |
| `BMI088_ReadGyroX/Y/Z_LSB/DPS(...)` | Per-axis variants |

**Gyro ranges & sensitivity:**

| Range | Sensitivity (LSB/°/s) |
|---|---|
| ±2000 °/s | 16.384 |
| ±1000 °/s | 32.768 |
| ±500 °/s | 65.536 |
| ±250 °/s | 131.072 |
| ±125 °/s | 262.144 |

---

### Accelerometer

| Function | Description |
|---|---|
| `BMI088_InitAccel(sensor, hspi, port, pin)` | Full startup sequence, verify chip ID (`0x1E`), set defaults |
| `BMI088_ResetAccel(sensor)` | Soft reset |
| `BMI088_SetRangeAccel(sensor, range)` | Set range, auto-update sensitivity |
| `BMI088_SetODRAccel(sensor, odr)` | 12.5 – 1600 Hz |
| `BMI088_SetBandwidthAccel(sensor, bwp)` | `OSR4` / `OSR2` / `NORMAL` |
| `BMI088_SwitchAccelLPM(sensor, mode)` | `ACTIVE` / `SUSPEND` |
| `BMI088_SelfTestAccel(sensor)` | Pos/neg excitation test per datasheet |
| `BMI088_ReadAccelXYZ_LSB(sensor, *raw)` | 3-axis burst, raw int16 |
| `BMI088_ReadAccelXYZ_MG(sensor, *data)` | 3-axis burst, float g |
| `BMI088_ReadAccelX/Y/Z_LSB/MG(...)` | Per-axis variants |

**Accel ranges & sensitivity:**

| Range | Sensitivity (LSB/g) |
|---|---|
| ±3g | 10920.0 |
| ±6g | 5460.0 |
| ±12g | 2730.0 |
| ±24g | 1365.0 |

---

### Temperature & Low-Level

```c
BMI088_ReadTemp_Raw(sensor, *dest);       // Raw int16
BMI088_ReadTemp_Celsius(sensor, *dest);   // Float °C (from accel registers)

// Direct register access (used internally, available for custom use)
BMI088_ReadReg(sensor, sensorType, regAddr, *data);
BMI088_ReadRegBurst(sensor, sensorType, regAddr, *dest);  // Returns int16
BMI088_WriteReg(sensor, sensorType, regAddr, data);
```

---

## SPI Protocol Notes

**Accelerometer reads require a dummy byte.** After the address byte, the first received byte is always discarded — real data starts at byte 2. Single reads are 3-byte transfers; the 3-axis burst receives 7 bytes (1 dummy + 6 data). The driver handles this automatically.

**Gyroscope reads are standard** — no dummy byte. 3-axis burst is exactly 6 bytes.

**Accelerometer SPI activation.** On power-up, the accelerometer defaults to I2C mode. `BMI088_InitAccel` pulses CS low→high to force SPI mode, then performs a mandatory dummy read before the real chip ID check.

**Write timing.** Every write includes a 1 ms delay for BMI088 settling.

**R/W bit:** `addr | 0x80` for reads, `addr & 0x7F` for writes.

---

## Data Structures

```c
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *accelCSPort; uint16_t accelCSPin;
    GPIO_TypeDef *gyroCSPort;  uint16_t gyroCSPin;
    // Populated automatically by Init and Set functions:
    float gyroSensitivity;   // LSB / (°/s)
    float accelSensitivity;  // LSB / g
    // ... range, ODR, BWP, LPM fields
} BMI088_t;

typedef struct { int16_t x, y, z; } BMI088_RawData_t;
typedef struct { float   x, y, z; } BMI088_FloatData_t;
```

Return type `BMI088_Status_t`: `BMI088_GYRO_OK`, `BMI088_GYRO_FAIL`, `BMI088_ACCEL_OK`, `BMI088_ACCEL_FAIL`.

---

## Resources

- [BMI088 Datasheet (BST-BMI088-DS001)](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- License: MIT — see [LICENSE](LICENSE)
