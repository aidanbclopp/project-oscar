#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#define TICKS_PER_INCH   106.86f
#define TICKS_PER_DEGREE 2.800f

#define MOTOR_VREF_MIN    70
#define MOTOR_VREF_MAX    200

#define RIGHT_MOTOR_TRIM  0.757f
#define CCW_TURN_TRIM     0.650f

#define HEADING_HOLD_KP       24.0f
#define HEADING_HOLD_KD       2.0f
#define HEADING_CORR_MAX      420.0f
#define ROTATE_KP             10.0f
#define ROTATE_MIN_SPEED      220
#define HEADING_TOL_DEG       2.0f
#define IMU_READ_PERIOD_MS    0U
#define HEADING_CTRL_TIMEOUT_MS 15000U
#define BNO055_MIN_GYRO_CALIB 2U

#endif