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
    wrapped_delta = (int16_t)(curr_ticks_u - start_ticks_u);
    return (int32_t)wrapped_delta;
}

static float NormalizeAngleErrorDeg(float err_deg)
{
    while (err_deg > 180.0f) {
        err_deg -= 360.0f;
    }
    while (err_deg < -180.0f) {
        err_deg += 360.0f;
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

void HeadingControl_GetDefaultConfig(HeadingControlConfig_t* cfg)
{
    if (cfg == NULL) {
        return;
    }

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
    HAL_TIM_PWM_Start(motor->htim, motor->ch_en);
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

    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_en, abs(speed));
}

void Motor_Stop(Motor_t* motor)
{
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_bwd, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->ch_en, 0);
}

void Motor_ResetDistance(Motor_t* motor)
{
    __HAL_TIM_SET_COUNTER(motor->enc_timer, 0);
    motor->last_position = 0;
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
    Motor_SetSpeed(motor_r, commanded_speed);

    while (average_dist < target_ticks) {
        int32_t curr_l = labs(Motor_GetSignedDeltaTicks(motor_l, start_l));
        int32_t curr_r = labs(Motor_GetSignedDeltaTicks(motor_r, start_r));
        average_dist = (curr_l + curr_r) / 2;
    }

    Motor_Stop(motor_l);
    Motor_Stop(motor_r);
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
        int32_t curr_l = labs(Motor_GetSignedDeltaTicks(motor_l, start_l));
        int32_t curr_r = labs(Motor_GetSignedDeltaTicks(motor_r, start_r));
        average_dist = (curr_l + curr_r) / 2;
    }

    Motor_Stop(motor_l);
    Motor_Stop(motor_r);
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

    if ((motor_l == NULL) || (motor_r == NULL) || (imu == NULL) || (cfg == NULL) || (speed == 0)) {
        printf("[CTRL] move-hold invalid params\r\n");
        return ROBOT_CTRL_PARAM_ERROR;
    }

    target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    if (target_ticks <= 0) {
        printf("[CTRL] move-hold skipped target=0\r\n");
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
        int32_t curr_l;
        int32_t curr_r;

        if (BNO055_GetHeadingDeg(imu, &curr_heading) != BNO055_OK) {
            Motor_Stop(motor_l);
            Motor_Stop(motor_r);
            printf("[CTRL] move-hold imu read failed in-loop\r\n");
            return ROBOT_CTRL_IMU_ERROR;
        }

        now_ms = HAL_GetTick();
        dt_s = (float)(now_ms - prev_tick_ms) / 1000.0f;
        if (dt_s < 0.001f) {
            dt_s = 0.001f;
        }

        heading_error = NormalizeAngleErrorDeg(target_heading - curr_heading);
        correction = (cfg->kp * heading_error) + (cfg->kd * ((heading_error - prev_error) / dt_s));
        correction = ClampFloat(correction, -cfg->corr_max, cfg->corr_max);

        left_cmd = (int32_t)((float)commanded_speed - correction);
        right_cmd = (int32_t)((float)commanded_speed + correction);
        Motor_SetSpeed(motor_l, ClampMotorSpeed(left_cmd));
        Motor_SetSpeed(motor_r, ClampMotorSpeed(right_cmd));

        prev_error = heading_error;
        prev_tick_ms = now_ms;

        curr_l = labs(Motor_GetSignedDeltaTicks(motor_l, start_l));
        curr_r = labs(Motor_GetSignedDeltaTicks(motor_r, start_r));
        average_dist = (curr_l + curr_r) / 2;

        if ((now_ms - start_ms) > cfg->timeout_ms) {
            Motor_Stop(motor_l);
            Motor_Stop(motor_r);
            printf("[CTRL] move-hold timeout elapsed=%lu targetTicks=%ld dist=%ld\r\n",
                   (unsigned long)(now_ms - start_ms),
                   (long)target_ticks,
                   (long)average_dist);
            return ROBOT_CTRL_TIMEOUT;
        }

        if ((now_ms - last_log_ms) >= 500U) {
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

    Motor_Stop(motor_l);
    Motor_Stop(motor_r);
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

    if ((motor_l == NULL) || (motor_r == NULL) || (imu == NULL) || (cfg == NULL) || (speed == 0)) {
        printf("[CTRL] rotate invalid params\r\n");
        return ROBOT_CTRL_PARAM_ERROR;
    }
    if (fabsf(delta_degrees) <= 0.01f) {
        printf("[CTRL] rotate skipped delta=0\r\n");
        return ROBOT_CTRL_OK;
    }

    if (BNO055_GetHeadingDeg(imu, &start_heading) != BNO055_OK) {
        Motor_Stop(motor_l);
        Motor_Stop(motor_r);
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
        float turn_cmd_f;
        int16_t turn_cmd;
        int32_t traveled_l;
        int32_t traveled_r;
        int32_t traveled_avg;
        uint32_t now_ms;

        if (BNO055_GetHeadingDeg(imu, &curr_heading) != BNO055_OK) {
            Motor_Stop(motor_l);
            Motor_Stop(motor_r);
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
        turn_cmd_f = cfg->rotate_kp * remaining_mag;
        if (turn_cmd_f < (float)cfg->rotate_min_speed) {
            turn_cmd_f = (float)cfg->rotate_min_speed;
        }
        if (turn_cmd_f > (float)max_turn_speed) {
            turn_cmd_f = (float)max_turn_speed;
        }
        turn_cmd_f *= direction_sign;

        turn_cmd = ClampMotorSpeed((int32_t)turn_cmd_f);
        Motor_SetSpeed(motor_l, (int16_t)(-turn_cmd));
        Motor_SetSpeed(motor_r, turn_cmd);

        traveled_l = labs(Motor_GetSignedDeltaTicks(motor_l, start_ticks_l));
        traveled_r = labs(Motor_GetSignedDeltaTicks(motor_r, start_ticks_r));
        traveled_avg = (traveled_l + traveled_r) / 2;
        now_ms = HAL_GetTick();

        if ((now_ms - start_ms) > cfg->timeout_ms) {
            Motor_Stop(motor_l);
            Motor_Stop(motor_r);
            printf("[CTRL] rotate timeout elapsed=%lu traveled=%ld accum=%d remain=%d\r\n",
                   (unsigned long)(now_ms - start_ms),
                   (long)traveled_avg,
                   (int)accumulated_delta,
                   (int)remaining_mag);
            return ROBOT_CTRL_TIMEOUT;
        }

        if ((now_ms - last_log_ms) >= 500U) {
            printf("[CTRL] rotate accum=%d remain=%d cmd=%d heading=%d\r\n",
                   (int)accumulated_delta,
                   (int)remaining_mag,
                   (int)turn_cmd,
                   (int)curr_heading);
            last_log_ms = now_ms;
        }
        HAL_Delay(cfg->sample_period_ms);
    }

    Motor_Stop(motor_l);
    Motor_Stop(motor_r);
    printf("[CTRL] rotate done accum=%d target=%d\r\n", (int)accumulated_delta, (int)delta_degrees);
    return ROBOT_CTRL_OK;
}