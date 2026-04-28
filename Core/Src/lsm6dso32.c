#include "lsm6dso32.h"

static HAL_StatusTypeDef reg_write(LSM6DSO32_Handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef reg_read(LSM6DSO32_Handle_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, &reg, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef LSM6DSO32_Init(LSM6DSO32_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    dev->hi2c              = hi2c;
    dev->i2c_addr          = addr;
    dev->gyro_sensitivity  = LSM6DSO32_GYRO_SENS_500;
    dev->accel_sensitivity = LSM6DSO32_ACCEL_SENS_4G;

    uint8_t who = 0;
    if (reg_read(dev, LSM6DSO32_WHO_AM_I, &who, 1) != HAL_OK) return HAL_ERROR;
    if (who != LSM6DSO32_WHO_AM_I_VAL)                         return HAL_ERROR;

    reg_write(dev, LSM6DSO32_CTRL3_C,  0x01);
    HAL_Delay(10);
    reg_write(dev, LSM6DSO32_CTRL2_G,  LSM6DSO32_ODR_104HZ | LSM6DSO32_GFS_500DPS);
    reg_write(dev, LSM6DSO32_CTRL1_XL, LSM6DSO32_ODR_104HZ | LSM6DSO32_AFS_4G);
    reg_write(dev, LSM6DSO32_CTRL3_C,  0x44);
    HAL_Delay(20);
    return HAL_OK;
}

HAL_StatusTypeDef LSM6DSO32_ReadAll(LSM6DSO32_Handle_t *dev, LSM6DSO32_Data_t *data)
{
    uint8_t raw[12];
    if (reg_read(dev, LSM6DSO32_OUTX_L_G, raw, 12) != HAL_OK) return HAL_ERROR;

    /* Gyro: bytes 0-5 */
    data->gx = (int16_t)((raw[1]  << 8) | raw[0])  * dev->gyro_sensitivity  / 1000.0f;
    data->gy = (int16_t)((raw[3]  << 8) | raw[2])  * dev->gyro_sensitivity  / 1000.0f;
    data->gz = (int16_t)((raw[5]  << 8) | raw[4])  * dev->gyro_sensitivity  / 1000.0f;

    /* Accel: bytes 6-11 — result in mg (milli-g)
     * Divide by 1000.0f to convert to g if preferred.        */
    data->ax = (int16_t)((raw[7]  << 8) | raw[6])  * dev->accel_sensitivity;
    data->ay = (int16_t)((raw[9]  << 8) | raw[8])  * dev->accel_sensitivity;
    data->az = (int16_t)((raw[11] << 8) | raw[10]) * dev->accel_sensitivity;

    return HAL_OK;
}

HAL_StatusTypeDef LSM6DSO32_ReadGyro(LSM6DSO32_Handle_t *dev, float *gx, float *gy, float *gz)
{
    uint8_t raw[6];
    if (reg_read(dev, LSM6DSO32_OUTX_L_G, raw, 6) != HAL_OK) return HAL_ERROR;

    *gx = (int16_t)((raw[1] << 8) | raw[0]) * dev->gyro_sensitivity / 1000.0f;
    *gy = (int16_t)((raw[3] << 8) | raw[2]) * dev->gyro_sensitivity / 1000.0f;
    *gz = (int16_t)((raw[5] << 8) | raw[4]) * dev->gyro_sensitivity / 1000.0f;
    return HAL_OK;
}
