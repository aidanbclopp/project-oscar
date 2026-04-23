#include "bno055.h"

#include <stdio.h>
#include <string.h>

#define BNO055_TIMEOUT_MS 50U

#define BNO055_REG_CHIP_ID      0x00U
#define BNO055_REG_OPR_MODE     0x3DU
#define BNO055_REG_PWR_MODE     0x3EU
#define BNO055_REG_SYS_TRIGGER  0x3FU
#define BNO055_REG_UNIT_SEL     0x3BU
#define BNO055_REG_CALIB_STAT   0x35U

#define BNO055_REG_EUL_H_LSB    0x1AU
#define BNO055_REG_QUA_DATA_W_LSB 0x20U

#define BNO055_CHIP_ID          0xA0U

#define BNO055_PWR_NORMAL       0x00U
#define BNO055_MODE_CONFIG      0x00U
#define BNO055_MODE_NDOF        0x0CU
#define BNO055_SYS_TRIGGER_RST  0x20U
#define BNO055_UNIT_SEL_DEG     0x00U

#define BNO055_BOOT_RETRY_DELAY_MS   650U
#define BNO055_RESET_SETTLE_DELAY_MS 700U
#define BNO055_CONFIG_MODE_DELAY_MS  20U
#define BNO055_PWR_MODE_DELAY_MS     10U
#define BNO055_UNIT_SEL_DELAY_MS     10U

static float BNO055_NormalizeSignedDeg(float deg)
{
    while (deg >= 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static BNO055_Status_t BNO055_WriteReg(BNO055_t* imu, uint8_t reg, uint8_t value)
{
    if (HAL_I2C_Mem_Write(imu->hi2c, imu->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, BNO055_TIMEOUT_MS) != HAL_OK) {
        return BNO055_ERR_I2C;
    }
    return BNO055_OK;
}

static BNO055_Status_t BNO055_ReadRegs(BNO055_t* imu, uint8_t reg, uint8_t* data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(imu->hi2c, imu->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, BNO055_TIMEOUT_MS) != HAL_OK) {
        return BNO055_ERR_I2C;
    }
    return BNO055_OK;
}

static void BNO055_SetMountConfig(BNO055_t* imu, const BNO055_MountConfig_t* mount_cfg)
{
    BNO055_MountConfig_t default_cfg;

    if (mount_cfg != NULL) {
        imu->mount_cfg = *mount_cfg;
    } else {
        BNO055_GetDefaultMountConfig(&default_cfg);
        imu->mount_cfg = default_cfg;
    }

    if (imu->mount_cfg.heading_sign == 0) {
        imu->mount_cfg.heading_sign = 1;
    }
}

static BNO055_Status_t BNO055_ReadAndCheckChipId(BNO055_t* imu, const char* fail_tag)
{
    uint8_t chip_id = 0U;

    if (BNO055_ReadRegs(imu, BNO055_REG_CHIP_ID, &chip_id, 1U) != BNO055_OK) {
        printf("[BNO055] %s chip-id read failed\r\n", fail_tag);
        return BNO055_ERR_I2C;
    }
    if (chip_id != BNO055_CHIP_ID) {
        printf("[BNO055] %s unexpected chip-id=0x%02X\r\n", fail_tag, chip_id);
        return BNO055_ERR_CHIP_ID;
    }
    return BNO055_OK;
}

static BNO055_Status_t BNO055_EnterConfigMode(BNO055_t* imu)
{
    if (BNO055_WriteReg(imu, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(BNO055_CONFIG_MODE_DELAY_MS);
    return BNO055_OK;
}

static BNO055_Status_t BNO055_ResetAndWait(BNO055_t* imu)
{
    if (BNO055_WriteReg(imu, BNO055_REG_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(BNO055_RESET_SETTLE_DELAY_MS);
    return BNO055_OK;
}

static BNO055_Status_t BNO055_ConfigureUnitsAndPower(BNO055_t* imu)
{
    if (BNO055_WriteReg(imu, BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(BNO055_PWR_MODE_DELAY_MS);

    if (BNO055_WriteReg(imu, BNO055_REG_UNIT_SEL, BNO055_UNIT_SEL_DEG) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(BNO055_UNIT_SEL_DELAY_MS);
    return BNO055_OK;
}

void BNO055_GetDefaultMountConfig(BNO055_MountConfig_t* cfg)
{
    if (cfg == NULL) {
        return;
    }

    cfg->heading_axis = BNO055_AXIS_HEADING;
    cfg->heading_sign = 1;
    cfg->heading_offset_deg = 0.0f;
}

BNO055_Status_t BNO055_SetModeNDOF(BNO055_t* imu)
{
    if ((imu == NULL) || (imu->hi2c == NULL)) {
        return BNO055_ERR_PARAM;
    }

    if (BNO055_WriteReg(imu, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(BNO055_CONFIG_MODE_DELAY_MS);

    if (BNO055_WriteReg(imu, BNO055_REG_OPR_MODE, BNO055_MODE_NDOF) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    HAL_Delay(BNO055_CONFIG_MODE_DELAY_MS);

    return BNO055_OK;
}

BNO055_Status_t BNO055_Init(BNO055_t* imu, I2C_HandleTypeDef* hi2c, uint16_t i2c_addr, const BNO055_MountConfig_t* mount_cfg)
{
    BNO055_Status_t st;

    if ((imu == NULL) || (hi2c == NULL)) {
        return BNO055_ERR_PARAM;
    }

    memset(imu, 0, sizeof(*imu));
    imu->hi2c = hi2c;
    imu->i2c_addr = i2c_addr;
    BNO055_SetMountConfig(imu, mount_cfg);

    printf("[BNO055] init start addr=0x%02X\r\n", (unsigned int)(i2c_addr >> 1));

    /* Init sequence mirrors datasheet flow: identify -> reset -> configure -> run. */
    st = BNO055_ReadAndCheckChipId(imu, "initial");
    if (st == BNO055_ERR_CHIP_ID) {
        HAL_Delay(BNO055_BOOT_RETRY_DELAY_MS);
        st = BNO055_ReadAndCheckChipId(imu, "retry");
    }
    if (st != BNO055_OK) {
        return st;
    }

    if (BNO055_EnterConfigMode(imu) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    if (BNO055_ResetAndWait(imu) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }
    st = BNO055_ReadAndCheckChipId(imu, "post-reset");
    if (st != BNO055_OK) {
        return st;
    }
    if (BNO055_ConfigureUnitsAndPower(imu) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }

    if (BNO055_SetModeNDOF(imu) != BNO055_OK) {
        printf("[BNO055] failed to enter NDOF mode\r\n");
        return BNO055_ERR_I2C;
    }

    imu->initialized = true;
    printf("[BNO055] init ok mode=NDOF axis=%d sign=%d\r\n",
           (int)imu->mount_cfg.heading_axis,
           (int)imu->mount_cfg.heading_sign);
    return BNO055_OK;
}

BNO055_Status_t BNO055_ReadEulerDeg(BNO055_t* imu, BNO055_EulerDeg_t* euler_deg)
{
    uint8_t raw[6];
    int16_t heading_raw;
    int16_t roll_raw;
    int16_t pitch_raw;

    if ((imu == NULL) || (euler_deg == NULL) || !imu->initialized) {
        return BNO055_ERR_PARAM;
    }

    if (BNO055_ReadRegs(imu, BNO055_REG_EUL_H_LSB, raw, sizeof(raw)) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }

    heading_raw = (int16_t)((raw[1] << 8) | raw[0]);
    roll_raw = (int16_t)((raw[3] << 8) | raw[2]);
    pitch_raw = (int16_t)((raw[5] << 8) | raw[4]);

    euler_deg->heading_deg = (float)heading_raw / 16.0f;
    euler_deg->roll_deg = (float)roll_raw / 16.0f;
    euler_deg->pitch_deg = (float)pitch_raw / 16.0f;

    return BNO055_OK;
}

BNO055_Status_t BNO055_ReadQuat(BNO055_t* imu, BNO055_Quat_t* quat)
{
    uint8_t raw[8];
    int16_t w_raw;
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    const float scale = 1.0f / 16384.0f;

    if ((imu == NULL) || (quat == NULL) || !imu->initialized) {
        return BNO055_ERR_PARAM;
    }

    if (BNO055_ReadRegs(imu, BNO055_REG_QUA_DATA_W_LSB, raw, sizeof(raw)) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }

    w_raw = (int16_t)((raw[1] << 8) | raw[0]);
    x_raw = (int16_t)((raw[3] << 8) | raw[2]);
    y_raw = (int16_t)((raw[5] << 8) | raw[4]);
    z_raw = (int16_t)((raw[7] << 8) | raw[6]);

    quat->w = (float)w_raw * scale;
    quat->x = (float)x_raw * scale;
    quat->y = (float)y_raw * scale;
    quat->z = (float)z_raw * scale;

    return BNO055_OK;
}

BNO055_Status_t BNO055_ReadCalibStatus(BNO055_t* imu, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
    uint8_t calib = 0;

    if ((imu == NULL) || !imu->initialized) {
        return BNO055_ERR_PARAM;
    }

    if (BNO055_ReadRegs(imu, BNO055_REG_CALIB_STAT, &calib, 1U) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }

    if (sys != NULL) {
        *sys = (calib >> 6) & 0x03U;
    }
    if (gyro != NULL) {
        *gyro = (calib >> 4) & 0x03U;
    }
    if (accel != NULL) {
        *accel = (calib >> 2) & 0x03U;
    }
    if (mag != NULL) {
        *mag = calib & 0x03U;
    }

    return BNO055_OK;
}

BNO055_Status_t BNO055_GetHeadingDeg(BNO055_t* imu, float* heading_deg)
{
    BNO055_EulerDeg_t euler;
    float selected = 0.0f;

    if ((imu == NULL) || (heading_deg == NULL) || !imu->initialized) {
        return BNO055_ERR_PARAM;
    }

    if (BNO055_ReadEulerDeg(imu, &euler) != BNO055_OK) {
        return BNO055_ERR_I2C;
    }

    switch (imu->mount_cfg.heading_axis) {
        case BNO055_AXIS_ROLL:
            selected = euler.roll_deg;
            break;
        case BNO055_AXIS_PITCH:
            selected = euler.pitch_deg;
            break;
        case BNO055_AXIS_HEADING:
        default:
            selected = euler.heading_deg;
            break;
    }

    selected *= (imu->mount_cfg.heading_sign >= 0) ? 1.0f : -1.0f;
    selected += imu->mount_cfg.heading_offset_deg;
    *heading_deg = BNO055_NormalizeSignedDeg(selected);

    return BNO055_OK;
}
