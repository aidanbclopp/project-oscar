#ifndef __BNO055_H__
#define __BNO055_H__

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* HAL I2C expects 8-bit addresses (7-bit device address shifted left by one). */
#define BNO055_I2C_ADDR_LOW  (0x28U << 1)
#define BNO055_I2C_ADDR_HIGH (0x29U << 1)

typedef enum {
    BNO055_OK = 0,
    BNO055_ERR_PARAM,
    BNO055_ERR_I2C,
    BNO055_ERR_CHIP_ID
} BNO055_Status_t;

typedef enum {
    BNO055_AXIS_HEADING = 0,
    BNO055_AXIS_ROLL = 1,
    BNO055_AXIS_PITCH = 2
} BNO055_Axis_t;

typedef struct {
    /* Heading is selected from one Euler axis, then sign/offset adjusted. */
    BNO055_Axis_t heading_axis;
    int8_t heading_sign;
    float heading_offset_deg;
} BNO055_MountConfig_t;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint16_t i2c_addr;
    BNO055_MountConfig_t mount_cfg;
    bool initialized;
} BNO055_t;

typedef struct {
    float heading_deg;
    float roll_deg;
    float pitch_deg;
} BNO055_EulerDeg_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} BNO055_Quat_t;

void BNO055_GetDefaultMountConfig(BNO055_MountConfig_t* cfg);
BNO055_Status_t BNO055_Init(BNO055_t* imu, I2C_HandleTypeDef* hi2c, uint16_t i2c_addr, const BNO055_MountConfig_t* mount_cfg);
BNO055_Status_t BNO055_SetModeNDOF(BNO055_t* imu);
BNO055_Status_t BNO055_ReadEulerDeg(BNO055_t* imu, BNO055_EulerDeg_t* euler_deg);
BNO055_Status_t BNO055_ReadQuat(BNO055_t* imu, BNO055_Quat_t* quat);
BNO055_Status_t BNO055_ReadCalibStatus(BNO055_t* imu, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
BNO055_Status_t BNO055_GetHeadingDeg(BNO055_t* imu, float* heading_deg);

#endif /* __BNO055_H__ */
