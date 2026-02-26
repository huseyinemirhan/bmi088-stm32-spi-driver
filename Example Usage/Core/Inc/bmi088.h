/*
 * bmi088.h
 *
 *  Created on: Feb 3, 2026
 *      Author: H. Emirhan Solak
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include <stdint.h>
#include "stm32h5xx_hal.h"

#define READ_OP  0x80
#define WRITE_OP 0x7F

/* Gyro Registers */
#define GYRO_CHIP_ID_REG            0x00
#define GYRO_RATE_X_LSB_REG         0x02
#define GYRO_RATE_Y_LSB_REG         0x04
#define GYRO_RATE_Z_LSB_REG         0x06
#define GYRO_INT_STAT1_REG          0x0A
#define FIFO_STATUS_REG             0x0E
#define GYRO_RANGE_REG              0x0F
#define GYRO_BANDWITH_REG           0x10
#define GYRO_LPM1_REG               0x11
#define GYRO_SOFT_RESET_REG         0x14
#define GYRO_INT_CTRL_REG           0x15
#define GYRO_INT3_INT4_IO_CONF_REG  0x16
#define GYRO_INT3_INT4_IO_MAP_REG   0x18
#define GYRO_FIFO_WM_ENABLE_REG     0x1E
#define GYRO_FIFO_EXT_INT_S_REG     0x34
#define GYRO_GYRO_SELF_TEST_REG     0x3C
#define GYRO_FIFO_CONFIG_0_REG      0x3D
#define GYRO_FIFO_CONFIG_1_REG      0x3E
#define GYRO_FIFO_DATA_REG          0x3F

/* Accel Registers */
#define ACC_CHIP_ID_REG             0x00
#define ACC_ERR_REG                 0x02
#define ACC_STATUS_REG              0x03
#define ACC_X_LSB_REG               0x12
#define ACC_X_MSB_REG               0x13
#define ACC_Y_LSB_REG               0x14
#define ACC_Y_MSB_REG               0x15
#define ACC_Z_LSB_REG               0x16
#define ACC_Z_MSB_REG               0x17
#define SENSORTIME_0_REG            0x18
#define SENSORTIME_1_REG            0x19
#define SENSORTIME_2_REG            0x1A
#define ACC_INT_STAT_1_REG          0x1D
#define TEMP_MSB_REG                0x22
#define TEMP_LSB_REG                0x23
#define FIFO_LENGTH_0_REG           0x24
#define FIFO_LENGTH_1_REG           0x25
#define FIFO_DATA_REG               0x26
#define ACC_CONF_REG                0x40
#define ACC_RANGE_REG               0x41
#define FIFO_DOWNS_REG              0x45
#define FIFO_WTM_0_REG              0x46
#define FIFO_WTM_1_REG              0x47
#define FIFO_CONFIG_0_REG           0x48
#define FIFO_CONFIG_1_REG           0x49
#define INT1_IO_CTRL_REG            0x53
#define INT2_IO_CTRL_REG            0x54
#define INT_MAP_DATA_REG            0x58
#define ACC_SELF_TEST_REG           0x6D
#define ACC_PWR_CONF_REG            0x7C
#define ACC_PWR_CTRL_REG            0x7D
#define ACC_SOFTRESET_REG           0x7E

typedef enum {
    BMI088_GYRO_OK    = 0x01,
    BMI088_GYRO_FAIL  = 0x02,
    BMI088_ACCEL_OK   = 0x03,
    BMI088_ACCEL_FAIL = 0x04,
} BMI088_Status_t;

typedef enum {
    BMI088_GYRO_RANGE_2000 = 0x00,
    BMI088_GYRO_RANGE_1000 = 0x01,
    BMI088_GYRO_RANGE_500  = 0x02,
    BMI088_GYRO_RANGE_250  = 0x03,
    BMI088_GYRO_RANGE_125  = 0x04
} BMI088_GyroRange_t;

typedef enum {
    BMI088_GYRO_BW_2000_532 = 0x00,
    BMI088_GYRO_BW_2000_230 = 0x01,
    BMI088_GYRO_BW_1000_116 = 0x02,
    BMI088_GYRO_BW_400_47   = 0x03,
    BMI088_GYRO_BW_200_23   = 0x04,
    BMI088_GYRO_BW_100_12   = 0x05,
    BMI088_GYRO_BW_200_64   = 0x06,
    BMI088_GYRO_BW_100_32   = 0x07
} BMI088_GyroBandwidth_t;

typedef enum {
    BMI088_GYRO_LPM_NORMAL      = 0x00,
    BMI088_GYRO_LPM_SUSPEND     = 0x80,
    BMI088_GYRO_LPM_DEEPSUSPEND = 0x20
} BMI088_GyroLPM_t;

typedef enum {
    BMI088_ACCEL_RANGE_3G  = 0x00,
    BMI088_ACCEL_RANGE_6G  = 0x01,
    BMI088_ACCEL_RANGE_12G = 0x02,
    BMI088_ACCEL_RANGE_24G = 0x03
} BMI088_AccelRange_t;

typedef enum {
    BMI088_ACCEL_ODR_12_5 = 0x05,
    BMI088_ACCEL_ODR_25   = 0x06,
    BMI088_ACCEL_ODR_50   = 0x07,
    BMI088_ACCEL_ODR_100  = 0x08,
    BMI088_ACCEL_ODR_200  = 0x09,
    BMI088_ACCEL_ODR_400  = 0x0A,
    BMI088_ACCEL_ODR_800  = 0x0B,
    BMI088_ACCEL_ODR_1600 = 0x0C
} BMI088_AccelODR_t;

typedef enum {
    BMI088_ACCEL_LPM_ACTIVE  = 0x00,
    BMI088_ACCEL_LPM_SUSPEND = 0x03,
} BMI088_AccelLPM_t;

typedef enum {
    BMI088_ACCEL_BWP_OSR4   = 0x08,
    BMI088_ACCEL_BWP_OSR2   = 0x09,
    BMI088_ACCEL_BWP_NORMAL = 0x0A
} BMI088_AccelBWP_t;

typedef enum {
    BMI088_GYRO  = 0x01,
    BMI088_ACCEL = 0x02,
} BMI088_SensorType;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} BMI088_RawData_t;

typedef struct {
    float x;
    float y;
    float z;
} BMI088_FloatData_t;

typedef struct {

    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *accelCSPort;
    uint16_t      accelCSPin;

    GPIO_TypeDef *gyroCSPort;
    uint16_t      gyroCSPin;

    BMI088_GyroRange_t     gyroRange;
    BMI088_GyroBandwidth_t gyroBandwidth;
    float                  gyroSensitivity;
    BMI088_GyroLPM_t       gyroLPM;

    BMI088_AccelRange_t accelRange;
    BMI088_AccelODR_t   accelODR;
    BMI088_AccelBWP_t   accelBWP;
    float               accelSensitivity;
    BMI088_AccelLPM_t   accelLPM;

} BMI088_t;

/* General Functions */
HAL_StatusTypeDef BMI088_ReadReg(BMI088_t *sensor, BMI088_SensorType sensorType, uint8_t regAddr, uint8_t *regData);
HAL_StatusTypeDef BMI088_ReadRegBurst(BMI088_t *sensor, BMI088_SensorType sensorType, uint8_t regAddr, int16_t *dest);
HAL_StatusTypeDef BMI088_WriteReg(BMI088_t *sensor, BMI088_SensorType sensorType, uint8_t regAddr, uint8_t regData);

/* Gyro Functions */
BMI088_Status_t BMI088_InitGyro(BMI088_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin);
BMI088_Status_t BMI088_ResetGyro(BMI088_t *sensor);
BMI088_Status_t BMI088_SetRangeGyro(BMI088_t *sensor, BMI088_GyroRange_t range);
BMI088_Status_t BMI088_SetBandwithGyro(BMI088_t *sensor, BMI088_GyroBandwidth_t bandwidth);
BMI088_Status_t BMI088_SelfTestGyro(BMI088_t *sensor);
BMI088_Status_t BMI088_ReadChipID_Gyro(BMI088_t *sensor);
BMI088_Status_t BMI088_SwitchGyroLPM(BMI088_t *sensor, BMI088_GyroLPM_t mode);

BMI088_Status_t BMI088_ReadGyroX_LSB(BMI088_t *sensor, int16_t *dest);
BMI088_Status_t BMI088_ReadGyroY_LSB(BMI088_t *sensor, int16_t *dest);
BMI088_Status_t BMI088_ReadGyroZ_LSB(BMI088_t *sensor, int16_t *dest);
BMI088_Status_t BMI088_ReadGyroXYZ_LSB(BMI088_t *sensor, BMI088_RawData_t *rawData);

BMI088_Status_t BMI088_ReadGyroX_DPS(BMI088_t *sensor, float *dest);
BMI088_Status_t BMI088_ReadGyroY_DPS(BMI088_t *sensor, float *dest);
BMI088_Status_t BMI088_ReadGyroZ_DPS(BMI088_t *sensor, float *dest);
BMI088_Status_t BMI088_ReadGyroXYZ_DPS(BMI088_t *sensor, BMI088_FloatData_t *floatData);

/* Accel Functions */
BMI088_Status_t BMI088_InitAccel(BMI088_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin);
BMI088_Status_t BMI088_ResetAccel(BMI088_t *sensor);
BMI088_Status_t BMI088_SetRangeAccel(BMI088_t *sensor, BMI088_AccelRange_t range);
BMI088_Status_t BMI088_SetBandwidthAccel(BMI088_t *sensor, BMI088_AccelBWP_t bandwidth);
BMI088_Status_t BMI088_SetODRAccel(BMI088_t *sensor, BMI088_AccelODR_t odr);
BMI088_Status_t BMI088_SelfTestAccel(BMI088_t *sensor);
BMI088_Status_t BMI088_ReadChipID_Accel(BMI088_t *sensor);
BMI088_Status_t BMI088_SwitchAccelLPM(BMI088_t *sensor, BMI088_AccelLPM_t mode);

BMI088_Status_t BMI088_ReadAccelX_LSB(BMI088_t *sensor, int16_t *dest);
BMI088_Status_t BMI088_ReadAccelY_LSB(BMI088_t *sensor, int16_t *dest);
BMI088_Status_t BMI088_ReadAccelZ_LSB(BMI088_t *sensor, int16_t *dest);
BMI088_Status_t BMI088_ReadAccelXYZ_LSB(BMI088_t *sensor, BMI088_RawData_t *rawData);

BMI088_Status_t BMI088_ReadAccelX_MG(BMI088_t *sensor, float *dest);
BMI088_Status_t BMI088_ReadAccelY_MG(BMI088_t *sensor, float *dest);
BMI088_Status_t BMI088_ReadAccelZ_MG(BMI088_t *sensor, float *dest);
BMI088_Status_t BMI088_ReadAccelXYZ_MG(BMI088_t *sensor, BMI088_FloatData_t *floatData);

/* Temperature Functions */
BMI088_Status_t BMI088_ReadTemp_Raw(BMI088_t *sensor, int16_t *dest);
BMI088_Status_t BMI088_ReadTemp_Celsius(BMI088_t *sensor, float *dest);

#endif /* INC_BMI088_H_ */
