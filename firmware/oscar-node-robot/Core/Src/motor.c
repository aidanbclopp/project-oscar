/**
  ******************************************************************************
  * @file    motor.c
  * @brief   This file provides code for the configuration
  *          of all motor prototypes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Aidan Clopp.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "motor.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define ANGLE_HALF_TURN_DEG      180.0f
#define ANGLE_FULL_TURN_DEG      360.0f
#define SEC_PER_MS               1000.0f
#define CTRL_MIN_DT_SEC          0.001f
#define CTRL_LOG_PERIOD_MS       500U
#define CTRL_ZERO_DELTA_EPS_DEG  0.01f

static int16_t ClampMotorSpeed(int32_t speed)
{
    if (speed > PWM_PERIOD) {
        return PWM_PERIOD;
    }
    if (speed < -PWM_PERIOD) {
        return -PWM_PERIOD;
    }
    return (int16_t)speed;
}

static int32_t Motor_GetSignedDeltaTicks(Motor_t* motor, int16_t start_ticks)
{
    uint16_t curr_ticks_u;
    uint16_t start_ticks_u;
    int16_t wrapped_delta;

    curr_ticks_u = (uint16_t)__HAL_TIM_GET_COUNTER(motor->enc_timer);
    start_ticks_u = (uint16_t)start_ticks;
    /* Signed wraparound keeps short-window encoder deltas consistent. */
    wrapped_delta = (int16_t)(curr_ticks_u - start_ticks_u);
    return (int32_t)wrapped_delta;
}

static float NormalizeAngleErrorDeg(float err_deg)
{
    while (err_deg > ANGLE_HALF_TURN_DEG) {
        err_deg -= ANGLE_FULL_TURN_DEG;
    }
    while (err_deg < -ANGLE_HALF_TURN_DEG) {
        err_deg += ANGLE_FULL_TURN_DEG;
    }
    return err_deg;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void Motor_StopPair(Motor_t* motor_l, Motor_t* motor_r)
{
    Motor_Stop(motor_l);
    Motor_Stop(motor_r);
}

static int32_t Motor_GetAverageDistanceTicks(Motor_t* motor_l, int16_t start_l, Motor_t* motor_r, int16_t start_r)
{
    int32_t curr_l = labs(Motor_GetSignedDeltaTicks(motor_l, start_l));
    int32_t curr_r = labs(Motor_GetSignedDeltaTicks(motor_r, start_r));
    return (curr_l + curr_r) / 2;
}

static float Motor_GetDeltaSeconds(uint32_t now_ms, uint32_t prev_ms)
{
    float dt_s = (float)(now_ms - prev_ms) / SEC_PER_MS;
    if (dt_s < CTRL_MIN_DT_SEC) {
        dt_s = CTRL_MIN_DT_SEC;
    }
    return dt_s;
}

static float Motor_GetHeadingCorrectionSign(int16_t commanded_speed)
{
    /* Differential steering reverses when translating backwards. */
    return (commanded_speed >= 0) ? 1.0f : -1.0f;
}

static int16_t Motor_ApplyRightTrim(int32_t speed)
{
    return ClampMotorSpeed((int32_t)((float)speed * RIGHT_MOTOR_TRIM));
}

static int16_t Motor_ComputeRotateCommand(const HeadingControlConfig_t* cfg, int16_t max_turn_speed,
                                          float remaining_mag, float direction_sign)
{
    /* Proportional turn with floor/ceiling to avoid stall and overspeed. */
    float turn_cmd_f = cfg->rotate_kp * remaining_mag;
    if (turn_cmd_f < (float)cfg->rotate_min_speed) {
        turn_cmd_f = (float)cfg->rotate_min_speed;
    }
    if (turn_cmd_f > (float)max_turn_speed) {
        turn_cmd_f = (float)max_turn_speed;
    }

    if (direction_sign > 0.0f) {
        /* Counter-clockwise turns over-rotate on this drivetrain. */
        turn_cmd_f *= CCW_TURN_TRIM;
    }

    return ClampMotorSpeed((int32_t)(turn_cmd_f * direction_sign));
}

void HeadingControl_GetDefaultConfig(HeadingControlConfig_t* cfg)
{
    cfg->kp = HEADING_HOLD_KP;
    cfg->kd = HEADING_HOLD_KD;
    cfg->corr_max = HEADING_CORR_MAX;
    cfg->heading_tolerance_deg = HEADING_TOL_DEG;
    cfg->rotate_kp = ROTATE_KP;
    cfg->rotate_min_speed = ROTATE_MIN_SPEED;
    cfg->sample_period_ms = IMU_READ_PERIOD_MS;
    cfg->timeout_ms = HEADING_CTRL_TIMEOUT_MS;
}

void Motor_Init(Motor_t* motor)
{
    HAL_TIM_Encoder_Start(motor->enc_timer, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(motor->htim, motor->ch_fwd);
    HAL_TIM_PWM_Start(motor->htim, motor->ch_bwd);
    HAL_TIM_PWM_Start(motor->htim, motor->ch_pwm);
    Motor_Stop(motor);
}

void Motor_SetSpeed(Motor_t* motor, int16_t speed)
{
    if (speed == 0) {
        Motor_Stop(motor);
        return;
    }

    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_fwd, PWM_PERIOD);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_bwd, 0);
    } else {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_fwd, 0);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_bwd, PWM_PERIOD);
    }

    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_pwm, abs(speed));
}

void Motor_Stop(Motor_t* motor)
{
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_bwd, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_pwm, 0);
}

void Motor_ResetDistance(Motor_t* motor)
{
    __HAL_TIM_SET_COUNTER(motor->enc_timer, 0);
}

int32_t Motor_GetTicks(Motor_t* motor)
{
    return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(motor->enc_timer);
}

void Robot_MoveDistance(Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float inches)
{
    int32_t target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    int16_t start_l = (int16_t)Motor_GetTicks(motor_l);
    int16_t start_r = (int16_t)Motor_GetTicks(motor_r);
    int32_t average_dist = 0;
    int16_t commanded_speed = (inches < 0.0f) ? (int16_t)(-abs(speed)) : (int16_t)abs(speed);

    Motor_SetSpeed(motor_l, commanded_speed);
    Motor_SetSpeed(motor_r, Motor_ApplyRightTrim(commanded_speed));

    while (average_dist < target_ticks) {
        average_dist = Motor_GetAverageDistanceTicks(motor_l, start_l, motor_r, start_r);
    }

    Motor_StopPair(motor_l, motor_r);
}

void Robot_Rotate(Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float degrees)
{
    int32_t target_ticks = (int32_t)(fabsf(degrees) * TICKS_PER_DEGREE);
    int16_t start_l = (int16_t)Motor_GetTicks(motor_l);
    int16_t start_r = (int16_t)Motor_GetTicks(motor_r);
    int32_t average_dist = 0;
    int16_t turn_speed = (int16_t)abs(speed);

    if (degrees == 0.0f || turn_speed == 0) {
        return;
    }

    if (degrees > 0.0f) {
        Motor_SetSpeed(motor_l, -turn_speed);
        Motor_SetSpeed(motor_r, turn_speed);
    } else {
        Motor_SetSpeed(motor_l, turn_speed);
        Motor_SetSpeed(motor_r, -turn_speed);
    }

    while (average_dist < target_ticks) {
        average_dist = Motor_GetAverageDistanceTicks(motor_l, start_l, motor_r, start_r);
    }

    Motor_StopPair(motor_l, motor_r);
}

RobotControlStatus_t Robot_MoveDistanceHeadingHold(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float inches)
{
    int32_t target_ticks;
    int16_t start_l;
    int16_t start_r;
    int32_t average_dist = 0;
    int16_t commanded_speed;
    float target_heading;
    float prev_error = 0.0f;
    uint32_t prev_tick_ms;
    uint32_t start_ms;
    uint32_t last_log_ms = 0U;

    if (speed == 0) {
        return ROBOT_CTRL_OK;
    }

    target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    if (target_ticks <= 0) {
        return ROBOT_CTRL_OK;
    }

    commanded_speed = (inches < 0.0f) ? (int16_t)(-abs(speed)) : (int16_t)abs(speed);
    if (BNO055_GetHeadingDeg(imu, &target_heading) != BNO055_OK) {
        Motor_Stop(motor_l);
        Motor_Stop(motor_r);
        printf("[CTRL] move-hold imu read failed at start\r\n");
        return ROBOT_CTRL_IMU_ERROR;
    }
    printf("[CTRL] move-hold start inches=%d speed=%d targetTicks=%ld heading=%d\r\n",
           (int)inches,
           (int)commanded_speed,
           (long)target_ticks,
           (int)target_heading);

    start_l = (int16_t)Motor_GetTicks(motor_l);
    start_r = (int16_t)Motor_GetTicks(motor_r);
    start_ms = HAL_GetTick();
    prev_tick_ms = start_ms;

    while (average_dist < target_ticks) {
        float curr_heading = 0.0f;
        float heading_error;
        float correction;
        float dt_s;
        uint32_t now_ms;
        int32_t left_cmd;
        int32_t right_cmd;

        if (BNO055_GetHeadingDeg(imu, &curr_heading) != BNO055_OK) {
            Motor_StopPair(motor_l, motor_r);
            printf("[CTRL] move-hold imu read failed in-loop\r\n");
            return ROBOT_CTRL_IMU_ERROR;
        }

        now_ms = HAL_GetTick();
        dt_s = Motor_GetDeltaSeconds(now_ms, prev_tick_ms);

        heading_error = NormalizeAngleErrorDeg(curr_heading - target_heading);
        correction = (cfg->kp * heading_error) + (cfg->kd * ((heading_error - prev_error) / dt_s));
        correction *= Motor_GetHeadingCorrectionSign(commanded_speed);
        correction = ClampFloat(correction, -cfg->corr_max, cfg->corr_max);

        left_cmd = (int32_t)((float)commanded_speed - correction);
        right_cmd = (int32_t)((float)Motor_ApplyRightTrim(commanded_speed) + correction);
        Motor_SetSpeed(motor_l, ClampMotorSpeed(left_cmd));
        Motor_SetSpeed(motor_r, ClampMotorSpeed(right_cmd));

        prev_error = heading_error;
        prev_tick_ms = now_ms;

        average_dist = Motor_GetAverageDistanceTicks(motor_l, start_l, motor_r, start_r);

        if ((now_ms - start_ms) > cfg->timeout_ms) {
            Motor_StopPair(motor_l, motor_r);
            printf("[CTRL] move-hold timeout elapsed=%lu targetTicks=%ld dist=%ld\r\n",
                   (unsigned long)(now_ms - start_ms),
                   (long)target_ticks,
                   (long)average_dist);
            return ROBOT_CTRL_TIMEOUT;
        }

        if ((now_ms - last_log_ms) >= CTRL_LOG_PERIOD_MS) {
            printf("[CTRL] move-hold dist=%ld/%ld err=%d corr=%d l=%ld r=%ld\r\n",
                   (long)average_dist,
                   (long)target_ticks,
                   (int)heading_error,
                   (int)correction,
                   (long)left_cmd,
                   (long)right_cmd);
            last_log_ms = now_ms;
        }
        HAL_Delay(cfg->sample_period_ms);
    }

    Motor_StopPair(motor_l, motor_r);
    printf("[CTRL] move-hold done dist=%ld\r\n", (long)average_dist);
    return ROBOT_CTRL_OK;
}

RobotControlStatus_t Robot_RotateToDeltaHeading(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float delta_degrees)
{
    float start_heading = 0.0f;
    float prev_heading = 0.0f;
    float accumulated_delta = 0.0f;
    float target_magnitude;
    float direction_sign;
    int16_t start_ticks_l;
    int16_t start_ticks_r;
    uint32_t start_ms;
    int16_t max_turn_speed;
    uint32_t last_log_ms = 0U;

    if (speed == 0) {
        return ROBOT_CTRL_OK;
    }
    if (fabsf(delta_degrees) <= CTRL_ZERO_DELTA_EPS_DEG) {
        return ROBOT_CTRL_OK;
    }

    if (BNO055_GetHeadingDeg(imu, &start_heading) != BNO055_OK) {
        Motor_StopPair(motor_l, motor_r);
        printf("[CTRL] rotate imu read failed at start\r\n");
        return ROBOT_CTRL_IMU_ERROR;
    }

    prev_heading = start_heading;
    target_magnitude = fabsf(delta_degrees);
    direction_sign = (delta_degrees >= 0.0f) ? 1.0f : -1.0f;
    start_ticks_l = (int16_t)Motor_GetTicks(motor_l);
    start_ticks_r = (int16_t)Motor_GetTicks(motor_r);
    start_ms = HAL_GetTick();
    max_turn_speed = (int16_t)abs(speed);
    printf("[CTRL] rotate start delta=%d speed=%d targetAbs=%d\r\n",
           (int)delta_degrees,
           (int)max_turn_speed,
           (int)target_magnitude);

    while (1) {
        float curr_heading = 0.0f;
        float step_delta;
        float remaining_mag;
        int16_t turn_cmd;
        int32_t traveled_avg;
        uint32_t now_ms;

        if (BNO055_GetHeadingDeg(imu, &curr_heading) != BNO055_OK) {
            Motor_StopPair(motor_l, motor_r);
            printf("[CTRL] rotate imu read failed in-loop\r\n");
            return ROBOT_CTRL_IMU_ERROR;
        }

        step_delta = NormalizeAngleErrorDeg(curr_heading - prev_heading);
        accumulated_delta += step_delta;
        prev_heading = curr_heading;

        remaining_mag = target_magnitude - fabsf(accumulated_delta);
        if (remaining_mag <= cfg->heading_tolerance_deg) {
            break;
        }

        /* Keep turn direction fixed for the whole move to prevent oscillation. */
        turn_cmd = Motor_ComputeRotateCommand(cfg, max_turn_speed, remaining_mag, direction_sign);
        Motor_SetSpeed(motor_l, (int16_t)(-turn_cmd));
        Motor_SetSpeed(motor_r, turn_cmd);

        traveled_avg = Motor_GetAverageDistanceTicks(motor_l, start_ticks_l, motor_r, start_ticks_r);
        now_ms = HAL_GetTick();

        if ((now_ms - start_ms) > cfg->timeout_ms) {
            Motor_StopPair(motor_l, motor_r);
            printf("[CTRL] rotate timeout elapsed=%lu traveled=%ld accum=%d remain=%d\r\n",
                   (unsigned long)(now_ms - start_ms),
                   (long)traveled_avg,
                   (int)accumulated_delta,
                   (int)remaining_mag);
            return ROBOT_CTRL_TIMEOUT;
        }

        if ((now_ms - last_log_ms) >= CTRL_LOG_PERIOD_MS) {
            printf("[CTRL] rotate accum=%d remain=%d cmd=%d heading=%d\r\n",
                   (int)accumulated_delta,
                   (int)remaining_mag,
                   (int)turn_cmd,
                   (int)curr_heading);
            last_log_ms = now_ms;
        }
        HAL_Delay(cfg->sample_period_ms);
    }

    Motor_StopPair(motor_l, motor_r);
    printf("[CTRL] rotate done accum=%d target=%d\r\n", (int)accumulated_delta, (int)delta_degrees);
    return ROBOT_CTRL_OK;
}

/* --- Non-blocking motion runner (call Robot_MotionRunner_Tick from main loop) --- */

typedef enum {
    MOTION_IDLE = 0,
    MOTION_MOVE_ENC,
    MOTION_ROTATE_ENC,
    MOTION_MOVE_IMU,
    MOTION_ROTATE_IMU
} MotionMode_t;

static struct {
    MotionMode_t mode;
    Motor_t* motor_l;
    Motor_t* motor_r;
    BNO055_t* imu;
    const HeadingControlConfig_t* cfg;
    int32_t target_ticks;
    int16_t enc_start_l;
    int16_t enc_start_r;
    int16_t commanded_speed;
    int16_t max_turn_speed;
    float target_heading;
    float prev_error;
    uint32_t prev_tick_ms;
    uint32_t start_ms;
    uint32_t last_log_ms;
    float prev_heading;
    float accumulated_delta;
    float target_magnitude;
    float direction_sign;
    int16_t rot_trace_start_l;
    int16_t rot_trace_start_r;
} s_motion;

static uint8_t Motion_IsSampleDue(uint32_t now_ms, uint32_t prev_tick_ms, uint32_t sample_period_ms)
{
    return ((now_ms - prev_tick_ms) >= sample_period_ms) ? 1U : 0U;
}

static uint8_t Motion_HasTimedOut(uint32_t now_ms, uint32_t start_ms, uint32_t timeout_ms)
{
    return ((now_ms - start_ms) > timeout_ms) ? 1U : 0U;
}

static uint32_t Motion_GetPrevTickForImmediateSample(uint32_t now_ms, uint32_t sample_period_ms)
{
    return now_ms - sample_period_ms;
}

uint8_t Robot_MotionRunner_IsBusy(void)
{
    return (s_motion.mode != MOTION_IDLE) ? 1U : 0U;
}

uint8_t Robot_MotionRunner_IsUsingImu(void)
{
    return ((s_motion.mode == MOTION_MOVE_IMU) || (s_motion.mode == MOTION_ROTATE_IMU)) ? 1U : 0U;
}

void Robot_MotionRunner_Stop(void)
{
    if ((s_motion.motor_l != NULL) && (s_motion.motor_r != NULL)) {
        Motor_Stop(s_motion.motor_l);
        Motor_Stop(s_motion.motor_r);
    }
    s_motion.mode = MOTION_IDLE;
    s_motion.motor_l = NULL;
    s_motion.motor_r = NULL;
    s_motion.imu = NULL;
    s_motion.cfg = NULL;
}

RobotControlStatus_t Robot_MotionRunner_RequestMoveEnc(
    Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float inches)
{
    int32_t target_ticks;
    if (s_motion.mode != MOTION_IDLE) {
        return ROBOT_CTRL_PARAM_ERROR;
    }
    if (speed == 0) {
        return ROBOT_CTRL_OK;
    }
    target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    if (target_ticks <= 0) {
        return ROBOT_CTRL_OK;
    }
    s_motion.mode = MOTION_MOVE_ENC;
    s_motion.motor_l = motor_l;
    s_motion.motor_r = motor_r;
    s_motion.imu = NULL;
    s_motion.cfg = NULL;
    s_motion.target_ticks = target_ticks;
    s_motion.enc_start_l = (int16_t)Motor_GetTicks(motor_l);
    s_motion.enc_start_r = (int16_t)Motor_GetTicks(motor_r);
    s_motion.commanded_speed = (inches < 0.0f) ? (int16_t)(-abs(speed)) : (int16_t)abs(speed);
    Motor_SetSpeed(motor_l, s_motion.commanded_speed);
    Motor_SetSpeed(motor_r, Motor_ApplyRightTrim(s_motion.commanded_speed));
    return ROBOT_CTRL_OK;
}

RobotControlStatus_t Robot_MotionRunner_RequestRotateEnc(
    Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float degrees)
{
    int32_t target_ticks;
    int16_t turn_speed;
    if (s_motion.mode != MOTION_IDLE) {
        return ROBOT_CTRL_PARAM_ERROR;
    }
    turn_speed = (int16_t)abs(speed);
    if ((degrees == 0.0f) || (turn_speed == 0)) {
        return ROBOT_CTRL_OK;
    }
    target_ticks = (int32_t)(fabsf(degrees) * TICKS_PER_DEGREE);
    s_motion.mode = MOTION_ROTATE_ENC;
    s_motion.motor_l = motor_l;
    s_motion.motor_r = motor_r;
    s_motion.imu = NULL;
    s_motion.cfg = NULL;
    s_motion.target_ticks = target_ticks;
    s_motion.enc_start_l = (int16_t)Motor_GetTicks(motor_l);
    s_motion.enc_start_r = (int16_t)Motor_GetTicks(motor_r);
    s_motion.max_turn_speed = turn_speed;
    if (degrees > 0.0f) {
        Motor_SetSpeed(motor_l, -turn_speed);
        Motor_SetSpeed(motor_r, turn_speed);
    } else {
        Motor_SetSpeed(motor_l, turn_speed);
        Motor_SetSpeed(motor_r, -turn_speed);
    }
    return ROBOT_CTRL_OK;
}

RobotControlStatus_t Robot_MotionRunner_RequestMoveHeadingHold(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float inches)
{
    uint32_t t0;
    if (s_motion.mode != MOTION_IDLE) {
        return ROBOT_CTRL_PARAM_ERROR;
    }
    if (speed == 0) {
        return ROBOT_CTRL_OK;
    }
    s_motion.target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    if (s_motion.target_ticks <= 0) {
        return ROBOT_CTRL_OK;
    }
    s_motion.commanded_speed = (inches < 0.0f) ? (int16_t)(-abs(speed)) : (int16_t)abs(speed);
    if (BNO055_GetHeadingDeg(imu, &s_motion.target_heading) != BNO055_OK) {
        printf("[CTRL] motion req move-hold imu read failed\r\n");
        return ROBOT_CTRL_IMU_ERROR;
    }
    t0 = HAL_GetTick();
    s_motion.mode = MOTION_MOVE_IMU;
    s_motion.motor_l = motor_l;
    s_motion.motor_r = motor_r;
    s_motion.imu = imu;
    s_motion.cfg = cfg;
    s_motion.enc_start_l = (int16_t)Motor_GetTicks(motor_l);
    s_motion.enc_start_r = (int16_t)Motor_GetTicks(motor_r);
    s_motion.prev_error = 0.0f;
    s_motion.prev_tick_ms = Motion_GetPrevTickForImmediateSample(t0, cfg->sample_period_ms);
    s_motion.start_ms = t0;
    s_motion.last_log_ms = 0U;
    Motor_SetSpeed(motor_l, s_motion.commanded_speed);
    Motor_SetSpeed(motor_r, Motor_ApplyRightTrim(s_motion.commanded_speed));
    printf("[CTRL] motion move-hold start inches=%d speed=%d targetTicks=%ld heading=%d\r\n",
           (int)inches,
           (int)s_motion.commanded_speed,
           (long)s_motion.target_ticks,
           (int)s_motion.target_heading);
    return ROBOT_CTRL_OK;
}

RobotControlStatus_t Robot_MotionRunner_RequestRotateDeltaHeading(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float delta_degrees)
{
    uint32_t t0;
    float start_heading;
    if (s_motion.mode != MOTION_IDLE) {
        return ROBOT_CTRL_PARAM_ERROR;
    }
    if (speed == 0) {
        return ROBOT_CTRL_OK;
    }
    if (fabsf(delta_degrees) <= CTRL_ZERO_DELTA_EPS_DEG) {
        return ROBOT_CTRL_OK;
    }
    if (BNO055_GetHeadingDeg(imu, &start_heading) != BNO055_OK) {
        printf("[CTRL] motion req rotate imu read failed\r\n");
        return ROBOT_CTRL_IMU_ERROR;
    }
    t0 = HAL_GetTick();
    s_motion.mode = MOTION_ROTATE_IMU;
    s_motion.motor_l = motor_l;
    s_motion.motor_r = motor_r;
    s_motion.imu = imu;
    s_motion.cfg = cfg;
    s_motion.max_turn_speed = (int16_t)abs(speed);
    s_motion.prev_heading = start_heading;
    s_motion.accumulated_delta = 0.0f;
    s_motion.target_magnitude = fabsf(delta_degrees);
    s_motion.direction_sign = (delta_degrees >= 0.0f) ? 1.0f : -1.0f;
    s_motion.rot_trace_start_l = (int16_t)Motor_GetTicks(motor_l);
    s_motion.rot_trace_start_r = (int16_t)Motor_GetTicks(motor_r);
    s_motion.prev_tick_ms = Motion_GetPrevTickForImmediateSample(t0, cfg->sample_period_ms);
    s_motion.start_ms = t0;
    s_motion.last_log_ms = 0U;
    {
        int16_t turn_cmd = Motor_ComputeRotateCommand(
            cfg, s_motion.max_turn_speed, s_motion.target_magnitude, s_motion.direction_sign);
        Motor_SetSpeed(motor_l, (int16_t)(-turn_cmd));
        Motor_SetSpeed(motor_r, turn_cmd);
    }
    printf("[CTRL] motion rotate start delta=%d speed=%d targetAbs=%d\r\n",
           (int)delta_degrees,
           (int)s_motion.max_turn_speed,
           (int)s_motion.target_magnitude);
    return ROBOT_CTRL_OK;
}

void Robot_MotionRunner_Tick(void)
{
    switch (s_motion.mode) {
        case MOTION_IDLE:
            return;
        case MOTION_MOVE_ENC:
        case MOTION_ROTATE_ENC: {
            int32_t average_dist = Motor_GetAverageDistanceTicks(
                s_motion.motor_l, s_motion.enc_start_l, s_motion.motor_r, s_motion.enc_start_r);
            if (average_dist >= s_motion.target_ticks) {
                Motor_StopPair(s_motion.motor_l, s_motion.motor_r);
                Robot_MotionRunner_Stop();
            }
            return;
        }
        case MOTION_MOVE_IMU: {
            float curr_heading = 0.0f;
            float heading_error;
            float correction;
            float dt_s;
            uint32_t now_ms;
            int32_t left_cmd;
            int32_t right_cmd;
            int32_t average_dist;

            now_ms = HAL_GetTick();
            if (!Motion_IsSampleDue(now_ms, s_motion.prev_tick_ms, s_motion.cfg->sample_period_ms)) {
                return;
            }

            if (BNO055_GetHeadingDeg(s_motion.imu, &curr_heading) != BNO055_OK) {
                Motor_StopPair(s_motion.motor_l, s_motion.motor_r);
                printf("[CTRL] motion move-hold imu read failed\r\n");
                Robot_MotionRunner_Stop();
                return;
            }

            dt_s = Motor_GetDeltaSeconds(now_ms, s_motion.prev_tick_ms);

            heading_error = NormalizeAngleErrorDeg(curr_heading - s_motion.target_heading);
            correction = (s_motion.cfg->kp * heading_error) +
                         (s_motion.cfg->kd * ((heading_error - s_motion.prev_error) / dt_s));
            correction *= Motor_GetHeadingCorrectionSign(s_motion.commanded_speed);
            correction = ClampFloat(correction, -s_motion.cfg->corr_max, s_motion.cfg->corr_max);

            left_cmd = (int32_t)((float)s_motion.commanded_speed - correction);
            right_cmd = (int32_t)((float)Motor_ApplyRightTrim(s_motion.commanded_speed) + correction);
            Motor_SetSpeed(s_motion.motor_l, ClampMotorSpeed(left_cmd));
            Motor_SetSpeed(s_motion.motor_r, ClampMotorSpeed(right_cmd));

            s_motion.prev_error = heading_error;
            s_motion.prev_tick_ms = now_ms;

            average_dist = Motor_GetAverageDistanceTicks(
                s_motion.motor_l, s_motion.enc_start_l, s_motion.motor_r, s_motion.enc_start_r);

            if (average_dist >= s_motion.target_ticks) {
                Motor_StopPair(s_motion.motor_l, s_motion.motor_r);
                printf("[CTRL] motion move-hold done dist=%ld\r\n", (long)average_dist);
                Robot_MotionRunner_Stop();
                return;
            }

            if (Motion_HasTimedOut(now_ms, s_motion.start_ms, s_motion.cfg->timeout_ms)) {
                Motor_StopPair(s_motion.motor_l, s_motion.motor_r);
                printf("[CTRL] motion move-hold timeout dist=%ld\r\n", (long)average_dist);
                Robot_MotionRunner_Stop();
                return;
            }

            if ((now_ms - s_motion.last_log_ms) >= CTRL_LOG_PERIOD_MS) {
                printf("[CTRL] motion move-hold dist=%ld/%ld err=%d corr=%d l=%ld r=%ld\r\n",
                       (long)average_dist,
                       (long)s_motion.target_ticks,
                       (int)heading_error,
                       (int)correction,
                       (long)left_cmd,
                       (long)right_cmd);
                s_motion.last_log_ms = now_ms;
            }
            return;
        }
        case MOTION_ROTATE_IMU: {
            float curr_heading = 0.0f;
            float step_delta;
            float remaining_mag;
            int16_t turn_cmd;
            int32_t traveled_avg;
            uint32_t now_ms;

            now_ms = HAL_GetTick();
            if (!Motion_IsSampleDue(now_ms, s_motion.prev_tick_ms, s_motion.cfg->sample_period_ms)) {
                return;
            }

            if (BNO055_GetHeadingDeg(s_motion.imu, &curr_heading) != BNO055_OK) {
                Motor_StopPair(s_motion.motor_l, s_motion.motor_r);
                printf("[CTRL] motion rotate imu read failed\r\n");
                Robot_MotionRunner_Stop();
                return;
            }

            step_delta = NormalizeAngleErrorDeg(curr_heading - s_motion.prev_heading);
            s_motion.accumulated_delta += step_delta;
            s_motion.prev_heading = curr_heading;

            remaining_mag = s_motion.target_magnitude - fabsf(s_motion.accumulated_delta);
            if (remaining_mag <= s_motion.cfg->heading_tolerance_deg) {
                Motor_StopPair(s_motion.motor_l, s_motion.motor_r);
                printf("[CTRL] motion rotate done accum=%d target=%d\r\n",
                       (int)s_motion.accumulated_delta,
                       (int)(s_motion.target_magnitude * s_motion.direction_sign));
                Robot_MotionRunner_Stop();
                return;
            }

            turn_cmd = Motor_ComputeRotateCommand(
                s_motion.cfg, s_motion.max_turn_speed, remaining_mag, s_motion.direction_sign);
            Motor_SetSpeed(s_motion.motor_l, (int16_t)(-turn_cmd));
            Motor_SetSpeed(s_motion.motor_r, turn_cmd);

            s_motion.prev_tick_ms = now_ms;

            traveled_avg = Motor_GetAverageDistanceTicks(
                s_motion.motor_l, s_motion.rot_trace_start_l, s_motion.motor_r, s_motion.rot_trace_start_r);

            if (Motion_HasTimedOut(now_ms, s_motion.start_ms, s_motion.cfg->timeout_ms)) {
                Motor_StopPair(s_motion.motor_l, s_motion.motor_r);
                printf("[CTRL] motion rotate timeout traveled=%ld accum=%d remain=%d\r\n",
                       (long)traveled_avg,
                       (int)s_motion.accumulated_delta,
                       (int)remaining_mag);
                Robot_MotionRunner_Stop();
                return;
            }

            if ((now_ms - s_motion.last_log_ms) >= CTRL_LOG_PERIOD_MS) {
                printf("[CTRL] motion rotate accum=%d remain=%d cmd=%d heading=%d\r\n",
                       (int)s_motion.accumulated_delta,
                       (int)remaining_mag,
                       (int)turn_cmd,
                       (int)curr_heading);
                s_motion.last_log_ms = now_ms;
            }
            return;
        }
        default:
            Robot_MotionRunner_Stop();
            return;
    }
}