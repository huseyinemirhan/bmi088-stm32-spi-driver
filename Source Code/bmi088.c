/*
 * bmi088.c
 *
 *  Created on: Feb 3, 2026
 *      Author: H. Emirhan Solak
 */

#include "bmi088.h"
#include "stm32h5xx_hal.h"

BMI088_Status_t BMI088_InitGyro(BMI088_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin){
    sensor->hspi = hspi;
    sensor->gyroCSPort = CS_Port;
    sensor->gyroCSPin = CS_Pin;
    sensor->gyroLPM = BMI088_GYRO_LPM_NORMAL;
    HAL_GPIO_WritePin(sensor->gyroCSPort, sensor->gyroCSPin, 1);
    HAL_Delay(30);

    uint8_t chipID = BMI088_ReadChipID_Gyro(sensor);
    if(chipID != 0x0F) return BMI088_GYRO_FAIL;

    sensor->gyroRange = BMI088_GYRO_RANGE_2000;
    sensor->gyroSensitivity = 16.384f;

    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_ResetGyro(BMI088_t *sensor){
    HAL_StatusTypeDef status;
    uint8_t resetByte = 0xB6;
    status = BMI088_WriteReg(sensor, BMI088_GYRO, GYRO_SOFT_RESET_REG, resetByte);
    if(status != HAL_OK) return BMI088_GYRO_FAIL;
    HAL_Delay(30);

    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_SwitchGyroLPM(BMI088_t *sensor, BMI088_GyroLPM_t mode){
    HAL_StatusTypeDef status;

    status = BMI088_WriteReg(sensor, BMI088_GYRO, GYRO_LPM1_REG, (uint8_t)mode);
    if(status != HAL_OK) return BMI088_GYRO_FAIL;

    sensor->gyroLPM = mode;

    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_SetRangeGyro(BMI088_t *sensor, BMI088_GyroRange_t range){
    HAL_StatusTypeDef status;
    sensor->gyroRange = range;
    switch(range){
        case BMI088_GYRO_RANGE_2000:
            sensor->gyroSensitivity = 16.384f;
            break;
        case BMI088_GYRO_RANGE_1000:
            sensor->gyroSensitivity = 32.768f;
            break;
        case BMI088_GYRO_RANGE_500:
            sensor->gyroSensitivity = 65.536f;
            break;
        case BMI088_GYRO_RANGE_250:
            sensor->gyroSensitivity = 131.072f;
            break;
        case BMI088_GYRO_RANGE_125:
            sensor->gyroSensitivity = 262.144f;
            break;
        default:
            return BMI088_GYRO_FAIL;
    }

    status = BMI088_WriteReg(sensor, BMI088_GYRO, GYRO_RANGE_REG, (uint8_t)sensor->gyroRange);
    if(status != HAL_OK) return BMI088_GYRO_FAIL;
    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_SetBandwithGyro(BMI088_t *sensor, BMI088_GyroBandwidth_t bandwidth){
    HAL_StatusTypeDef status;

    status = BMI088_WriteReg(sensor, BMI088_GYRO, GYRO_BANDWITH_REG, (uint8_t)bandwidth);
    if(status != HAL_OK) return BMI088_GYRO_FAIL;
    sensor->gyroBandwidth = bandwidth;
    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_SelfTestGyro(BMI088_t *sensor){
    HAL_StatusTypeDef halStatus;
    uint8_t trig_bist = 0x01;
    uint8_t bist_rdy  = 0x02;
    uint8_t bist_fail = 0x04;

    halStatus = BMI088_WriteReg(sensor, BMI088_GYRO, GYRO_GYRO_SELF_TEST_REG, trig_bist);
    if(halStatus != HAL_OK) return BMI088_GYRO_FAIL;

    uint8_t selfTestReg = 0;
    uint32_t timeout = 100; // max 100ms
    uint32_t elapsed = 0;

    while(!(selfTestReg & bist_rdy)){
        halStatus = BMI088_ReadReg(sensor, BMI088_GYRO, GYRO_GYRO_SELF_TEST_REG, &selfTestReg);
        if(halStatus != HAL_OK) return BMI088_GYRO_FAIL;
        HAL_Delay(1);
        elapsed++;
        if(elapsed >= timeout) return BMI088_GYRO_FAIL;
    }

    if(selfTestReg & bist_fail){
        return BMI088_GYRO_FAIL;
    }

    return BMI088_GYRO_OK;
}

HAL_StatusTypeDef BMI088_ReadReg(BMI088_t *sensor, BMI088_SensorType sensorType, uint8_t regAddr, uint8_t *regData){

    HAL_StatusTypeDef status;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
    uint8_t rxData[3];
    uint8_t txData[] = {regAddr | READ_OP, 0x00, 0x00};

    if(sensorType == BMI088_GYRO){
        csPort = sensor->gyroCSPort;
        csPin  = sensor->gyroCSPin;
    } else {
        csPort = sensor->accelCSPort;
        csPin  = sensor->accelCSPin;
    }

    HAL_GPIO_WritePin(csPort, csPin, 0);

    if(sensorType == BMI088_ACCEL){
        status = HAL_SPI_TransmitReceive(sensor->hspi, txData, rxData, 3, 100);
        HAL_GPIO_WritePin(csPort, csPin, 1);
        if(status != HAL_OK) return status;
        *regData = rxData[2];
    } else {
        status = HAL_SPI_Transmit(sensor->hspi, txData, 1, 100);
        if(status != HAL_OK){
            HAL_GPIO_WritePin(csPort, csPin, 1);
            return status;
        }
        status = HAL_SPI_Receive(sensor->hspi, rxData, 1, 100);
        HAL_GPIO_WritePin(csPort, csPin, 1);
        if(status != HAL_OK) return status;
        *regData = rxData[0];
    }
    return status;
}

HAL_StatusTypeDef BMI088_ReadRegBurst(BMI088_t *sensor, BMI088_SensorType sensorType, uint8_t regAddr, int16_t *dest){

    HAL_StatusTypeDef status;
    uint8_t txData = regAddr | READ_OP;
    uint8_t data[2];
    GPIO_TypeDef *csPort;
    uint16_t csPin;

    if(sensorType == BMI088_GYRO){
        csPort = sensor->gyroCSPort;
        csPin  = sensor->gyroCSPin;
    } else {
        csPort = sensor->accelCSPort;
        csPin  = sensor->accelCSPin;
    }

    HAL_GPIO_WritePin(csPort, csPin, 0);

    status = HAL_SPI_Transmit(sensor->hspi, &txData, 1, 100);
    if(status != HAL_OK){
        HAL_GPIO_WritePin(csPort, csPin, 1);
        return status;
    }

    if(sensorType == BMI088_ACCEL){
        uint8_t rxBuff[3];
        status = HAL_SPI_Receive(sensor->hspi, rxBuff, 3, 100);
        if(status != HAL_OK){
            HAL_GPIO_WritePin(csPort, csPin, 1);
            return status;
        }
        *dest = (int16_t)((rxBuff[2] << 8) | rxBuff[1]);
        HAL_GPIO_WritePin(csPort, csPin, 1);
        return status;
    }

    status = HAL_SPI_Receive(sensor->hspi, data, 2, 100);
    HAL_GPIO_WritePin(csPort, csPin, 1);

    if(status == HAL_OK){
        *dest = (int16_t)((data[1] << 8) | data[0]);
    }

    return status;
}

HAL_StatusTypeDef BMI088_WriteReg(BMI088_t *sensor, BMI088_SensorType sensorType, uint8_t regAddr, uint8_t regData){

    HAL_StatusTypeDef status;
    uint8_t txData[2] = {regAddr & WRITE_OP, regData};
    GPIO_TypeDef *csPort;
    uint16_t csPin;

    if(sensorType == BMI088_GYRO){
        csPort = sensor->gyroCSPort;
        csPin  = sensor->gyroCSPin;
    } else {
        csPort = sensor->accelCSPort;
        csPin  = sensor->accelCSPin;
    }

    HAL_GPIO_WritePin(csPort, csPin, 0);
    status = HAL_SPI_Transmit(sensor->hspi, txData, 2, 100);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    HAL_Delay(1);

    return status;
}

uint8_t BMI088_ReadChipID_Gyro(BMI088_t *sensor){
    uint8_t id;
    BMI088_ReadReg(sensor, BMI088_GYRO, GYRO_CHIP_ID_REG, &id);
    return id;
}

BMI088_Status_t BMI088_ReadGyroX_LSB(BMI088_t *sensor, int16_t *dest){
    HAL_StatusTypeDef status = BMI088_ReadRegBurst(sensor, BMI088_GYRO, GYRO_RATE_X_LSB_REG, dest);
    return (status == HAL_OK) ? BMI088_GYRO_OK : BMI088_GYRO_FAIL;
}

BMI088_Status_t BMI088_ReadGyroY_LSB(BMI088_t *sensor, int16_t *dest){
    HAL_StatusTypeDef status = BMI088_ReadRegBurst(sensor, BMI088_GYRO, GYRO_RATE_Y_LSB_REG, dest);
    return (status == HAL_OK) ? BMI088_GYRO_OK : BMI088_GYRO_FAIL;
}

BMI088_Status_t BMI088_ReadGyroZ_LSB(BMI088_t *sensor, int16_t *dest){
    HAL_StatusTypeDef status = BMI088_ReadRegBurst(sensor, BMI088_GYRO, GYRO_RATE_Z_LSB_REG, dest);
    return (status == HAL_OK) ? BMI088_GYRO_OK : BMI088_GYRO_FAIL;
}

BMI088_Status_t BMI088_ReadGyroXYZ_LSB(BMI088_t *sensor, BMI088_RawData_t *rawData){
    HAL_StatusTypeDef status;
    uint8_t rxData[6];
    uint8_t txData = GYRO_RATE_X_LSB_REG | READ_OP;

    HAL_GPIO_WritePin(sensor->gyroCSPort, sensor->gyroCSPin, 0);

    status = HAL_SPI_Transmit(sensor->hspi, &txData, 1, 100);
    if(status != HAL_OK){
        HAL_GPIO_WritePin(sensor->gyroCSPort, sensor->gyroCSPin, 1);
        return BMI088_GYRO_FAIL;
    }

    status = HAL_SPI_Receive(sensor->hspi, rxData, 6, 100);
    HAL_GPIO_WritePin(sensor->gyroCSPort, sensor->gyroCSPin, 1);
    if(status != HAL_OK) return BMI088_GYRO_FAIL;

    rawData->x = (int16_t)((rxData[1] << 8) | rxData[0]);
    rawData->y = (int16_t)((rxData[3] << 8) | rxData[2]);
    rawData->z = (int16_t)((rxData[5] << 8) | rxData[4]);

    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_ReadGyroX_DPS(BMI088_t *sensor, float *dest){
    BMI088_Status_t status;
    int16_t rawX;

    status = BMI088_ReadGyroX_LSB(sensor, &rawX);
    if(status != BMI088_GYRO_OK) return BMI088_GYRO_FAIL;

    *dest = rawX / sensor->gyroSensitivity;
    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_ReadGyroY_DPS(BMI088_t *sensor, float *dest){
    BMI088_Status_t status;
    int16_t rawY;

    status = BMI088_ReadGyroY_LSB(sensor, &rawY);
    if(status != BMI088_GYRO_OK) return BMI088_GYRO_FAIL;

    *dest = rawY / sensor->gyroSensitivity;
    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_ReadGyroZ_DPS(BMI088_t *sensor, float *dest){
    BMI088_Status_t status;
    int16_t rawZ;

    status = BMI088_ReadGyroZ_LSB(sensor, &rawZ);
    if(status != BMI088_GYRO_OK) return BMI088_GYRO_FAIL;

    *dest = rawZ / sensor->gyroSensitivity;
    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_ReadGyroXYZ_DPS(BMI088_t *sensor, BMI088_FloatData_t *floatData){
    BMI088_Status_t status;
    BMI088_RawData_t rawData;

    status = BMI088_ReadGyroXYZ_LSB(sensor, &rawData);
    if(status != BMI088_GYRO_OK) return BMI088_GYRO_FAIL;

    floatData->x = rawData.x / sensor->gyroSensitivity;
    floatData->y = rawData.y / sensor->gyroSensitivity;
    floatData->z = rawData.z / sensor->gyroSensitivity;

    return BMI088_GYRO_OK;
}

BMI088_Status_t BMI088_InitAccel(BMI088_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin){

    sensor->hspi = hspi;
    sensor->accelCSPort = CS_Port;
    sensor->accelCSPin  = CS_Pin;
    sensor->accelLPM    = BMI088_ACCEL_LPM_ACTIVE;
    uint8_t dummy;
    uint8_t chipID;

    HAL_GPIO_WritePin(sensor->accelCSPort, sensor->accelCSPin, 1);
    HAL_Delay(50);

    // Switch to SPI mode
    HAL_GPIO_WritePin(sensor->accelCSPort, sensor->accelCSPin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(sensor->accelCSPort, sensor->accelCSPin, 1);
    HAL_Delay(50);

    // Power Config Active
    if(BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_PWR_CONF_REG, 0x00) != HAL_OK){
        return BMI088_ACCEL_FAIL;
    }
    HAL_Delay(1);

    // Accel On
    if(BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_PWR_CTRL_REG, 0x04) != HAL_OK){
        return BMI088_ACCEL_FAIL;
    }
    HAL_Delay(50);

    // Dummy read required by BMI088 accel SPI protocol on first access
    BMI088_ReadReg(sensor, BMI088_ACCEL, ACC_CHIP_ID_REG, &dummy);

    if(BMI088_ReadReg(sensor, BMI088_ACCEL, ACC_CHIP_ID_REG, &chipID) != HAL_OK){
        return BMI088_ACCEL_FAIL;
    }

    if(chipID != 0x1E){
        return BMI088_ACCEL_FAIL;
    }

    if(BMI088_SetRangeAccel(sensor, BMI088_ACCEL_RANGE_6G)        != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;
    if(BMI088_SetBandwidthAccel(sensor, BMI088_ACCEL_BWP_NORMAL)   != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;
    if(BMI088_SetODRAccel(sensor, BMI088_ACCEL_ODR_100)            != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_ResetAccel(BMI088_t *sensor){
    HAL_StatusTypeDef status;
    uint8_t resetByte = 0xB6;

    status = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_SOFTRESET_REG, resetByte);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    return BMI088_ACCEL_OK;
}

uint8_t BMI088_ReadChipID_Accel(BMI088_t *sensor){
    HAL_StatusTypeDef status;
    uint8_t chipID;

    status = BMI088_ReadReg(sensor, BMI088_ACCEL, ACC_CHIP_ID_REG, &chipID);
    if(status != HAL_OK) return 0;

    return chipID;
}

BMI088_Status_t BMI088_SwitchAccelLPM(BMI088_t *sensor, BMI088_AccelLPM_t mode){
    HAL_StatusTypeDef status;

    status = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_PWR_CONF_REG, (uint8_t)mode);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    sensor->accelLPM = mode;

    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_SetBandwidthAccel(BMI088_t *sensor, BMI088_AccelBWP_t bandwidth){
    uint8_t currentReg;
    HAL_StatusTypeDef status;

    status = BMI088_ReadReg(sensor, BMI088_ACCEL, ACC_CONF_REG, &currentReg);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    uint8_t correctReg = currentReg | ((uint8_t)bandwidth << 4);
    status = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_CONF_REG, correctReg);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    sensor->accelBWP = bandwidth;
    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_SetODRAccel(BMI088_t *sensor, BMI088_AccelODR_t odr){
    uint8_t currentReg;
    HAL_StatusTypeDef status = BMI088_ReadReg(sensor, BMI088_ACCEL, ACC_CONF_REG, &currentReg);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    uint8_t correctReg = currentReg | ((uint8_t)odr);
    status = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_CONF_REG, correctReg);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    sensor->accelODR = odr;
    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_SetRangeAccel(BMI088_t *sensor, BMI088_AccelRange_t range){
    HAL_StatusTypeDef status;
    status = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_RANGE_REG, (uint8_t)range);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    sensor->accelRange = range;

    switch(range){
        case BMI088_ACCEL_RANGE_3G:
            sensor->accelSensitivity = 10920.0f;
            break;
        case BMI088_ACCEL_RANGE_6G:
            sensor->accelSensitivity = 5460.0f;
            break;
        case BMI088_ACCEL_RANGE_12G:
            sensor->accelSensitivity = 2730.0f;
            break;
        case BMI088_ACCEL_RANGE_24G:
            sensor->accelSensitivity = 1365.0f;
            break;
        default:
            return BMI088_ACCEL_FAIL;
    }
    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_SelfTestAccel(BMI088_t *sensor){
    HAL_StatusTypeDef statusHAL;
    BMI088_Status_t statusBMI088;
    uint8_t pos_test          = 0x0D;
    uint8_t neg_test          = 0x09;
    uint8_t disable_test      = 0x00;
    uint8_t normalSample_mode = 0xA7;

    BMI088_RawData_t pos_test_resp;
    BMI088_RawData_t neg_test_resp;

    statusBMI088 = BMI088_SetRangeAccel(sensor, BMI088_ACCEL_RANGE_24G);
    if(statusBMI088 != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    statusBMI088 = BMI088_SetODRAccel(sensor, BMI088_ACCEL_ODR_100);
    if(statusBMI088 != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    statusHAL = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_CONF_REG, normalSample_mode);
    if(statusHAL != HAL_OK) return BMI088_ACCEL_FAIL;
    HAL_Delay(2);

    statusHAL = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_SELF_TEST_REG, pos_test);
    if(statusHAL != HAL_OK) return BMI088_ACCEL_FAIL;
    HAL_Delay(50);

    statusBMI088 = BMI088_ReadAccelXYZ_LSB(sensor, &pos_test_resp);
    if(statusBMI088 != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    statusHAL = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_SELF_TEST_REG, neg_test);
    if(statusHAL != HAL_OK) return BMI088_ACCEL_FAIL;
    HAL_Delay(50);

    statusBMI088 = BMI088_ReadAccelXYZ_LSB(sensor, &neg_test_resp);
    if(statusBMI088 != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    statusHAL = BMI088_WriteReg(sensor, BMI088_ACCEL, ACC_SELF_TEST_REG, disable_test);
    if(statusHAL != HAL_OK) return BMI088_ACCEL_FAIL;
    HAL_Delay(50);

    int32_t diffX = (int32_t)pos_test_resp.x - (int32_t)neg_test_resp.x;
    int32_t diffY = (int32_t)pos_test_resp.y - (int32_t)neg_test_resp.y;
    int32_t diffZ = (int32_t)pos_test_resp.z - (int32_t)neg_test_resp.z;

    if(diffX < 1000 || diffY < 1000 || diffZ < 500) return BMI088_ACCEL_FAIL;

    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_ReadAccelX_LSB(BMI088_t *sensor, int16_t *dest){
    HAL_StatusTypeDef status = BMI088_ReadRegBurst(sensor, BMI088_ACCEL, ACC_X_LSB_REG, dest);
    return (status == HAL_OK) ? BMI088_ACCEL_OK : BMI088_ACCEL_FAIL;
}

BMI088_Status_t BMI088_ReadAccelY_LSB(BMI088_t *sensor, int16_t *dest){
    HAL_StatusTypeDef status = BMI088_ReadRegBurst(sensor, BMI088_ACCEL, ACC_Y_LSB_REG, dest);
    return (status == HAL_OK) ? BMI088_ACCEL_OK : BMI088_ACCEL_FAIL;
}

BMI088_Status_t BMI088_ReadAccelZ_LSB(BMI088_t *sensor, int16_t *dest){
    HAL_StatusTypeDef status = BMI088_ReadRegBurst(sensor, BMI088_ACCEL, ACC_Z_LSB_REG, dest);
    return (status == HAL_OK) ? BMI088_ACCEL_OK : BMI088_ACCEL_FAIL;
}

BMI088_Status_t BMI088_ReadAccelXYZ_LSB(BMI088_t *sensor, BMI088_RawData_t *rawData){
    HAL_StatusTypeDef status;
    uint8_t rxBuff[7];
    uint8_t txData = ACC_X_LSB_REG | READ_OP;

    HAL_GPIO_WritePin(sensor->accelCSPort, sensor->accelCSPin, 0);

    status = HAL_SPI_Transmit(sensor->hspi, &txData, 1, 100);
    if(status != HAL_OK){
        HAL_GPIO_WritePin(sensor->accelCSPort, sensor->accelCSPin, 1);
        return BMI088_ACCEL_FAIL;
    }

    status = HAL_SPI_Receive(sensor->hspi, rxBuff, 7, 100);
    HAL_GPIO_WritePin(sensor->accelCSPort, sensor->accelCSPin, 1);
    if(status != HAL_OK) return BMI088_ACCEL_FAIL;

    rawData->x = (int16_t)((rxBuff[2] << 8) | rxBuff[1]);
    rawData->y = (int16_t)((rxBuff[4] << 8) | rxBuff[3]);
    rawData->z = (int16_t)((rxBuff[6] << 8) | rxBuff[5]);

    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_ReadAccelX_MG(BMI088_t *sensor, float *dest){
    BMI088_Status_t status;
    int16_t rawX;

    status = BMI088_ReadAccelX_LSB(sensor, &rawX);
    if(status != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    *dest = (float)rawX / sensor->accelSensitivity;
    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_ReadAccelY_MG(BMI088_t *sensor, float *dest){
    BMI088_Status_t status;
    int16_t rawY;

    status = BMI088_ReadAccelY_LSB(sensor, &rawY);
    if(status != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    *dest = (float)rawY / sensor->accelSensitivity;
    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_ReadAccelZ_MG(BMI088_t *sensor, float *dest){
    BMI088_Status_t status;
    int16_t rawZ;

    status = BMI088_ReadAccelZ_LSB(sensor, &rawZ);
    if(status != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    *dest = (float)rawZ / sensor->accelSensitivity;
    return BMI088_ACCEL_OK;
}

BMI088_Status_t BMI088_ReadAccelXYZ_MG(BMI088_t *sensor, BMI088_FloatData_t *floatData){
    BMI088_Status_t status;
    BMI088_RawData_t rawData;

    status = BMI088_ReadAccelXYZ_LSB(sensor, &rawData);
    if(status != BMI088_ACCEL_OK) return BMI088_ACCEL_FAIL;

    floatData->x = rawData.x / sensor->accelSensitivity;
    floatData->y = rawData.y / sensor->accelSensitivity;
    floatData->z = rawData.z / sensor->accelSensitivity;

    return BMI088_ACCEL_OK;
}

