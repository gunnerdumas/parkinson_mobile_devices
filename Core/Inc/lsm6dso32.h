#ifndef LSM6DSO32_H
#define LSM6DSO32_H


#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * LSM6DSO32 Driver
 * I2C1: SCL → PB6 (pin 46)  SDA → PB7 (pin 47)
 * INT1 → PB5 (pin 45)       INT2 → PB4 (pin 44)
 */

#define LSM6DSO32_I2C_ADDR_LOW    (0x6A << 1)
#define LSM6DSO32_I2C_ADDR_HIGH   (0x6B << 1)
#define LSM6DSO32_DEFAULT_ADDR    LSM6DSO32_I2C_ADDR_LOW

#define LSM6DSO32_WHO_AM_I        0x0F
#define LSM6DSO32_WHO_AM_I_VAL    0x6C
#define LSM6DSO32_CTRL1_XL        0x10
#define LSM6DSO32_CTRL2_G         0x11
#define LSM6DSO32_CTRL3_C         0x12
#define LSM6DSO32_STATUS_REG      0x1E
#define LSM6DSO32_OUTX_L_G        0x22
#define LSM6DSO32_OUTX_L_A        0x28

#define LSM6DSO32_ODR_104HZ       0x40
#define LSM6DSO32_GFS_500DPS      0x04
#define LSM6DSO32_GYRO_SENS_500   17.5f


#define LSM6DSO32_AFS_4G          0x08
#define LSM6DSO32_ACCEL_SENS_4G   0.122f

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;
    float              gyro_sensitivity;
    float              accel_sensitivity;
} LSM6DSO32_Handle_t;


typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
} LSM6DSO32_Data_t;

HAL_StatusTypeDef LSM6DSO32_Init    (LSM6DSO32_Handle_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);
HAL_StatusTypeDef LSM6DSO32_ReadAll (LSM6DSO32_Handle_t *dev, LSM6DSO32_Data_t *data);
HAL_StatusTypeDef LSM6DSO32_ReadGyro(LSM6DSO32_Handle_t *dev, float *gx, float *gy, float *gz);

#endif
