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
#include <stdlib.h>

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
    return (int32_t)__HAL_TIM_GET_COUNTER(motor->enc_timer);
}

void Robot_MoveDistance(Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float inches)
{
    int32_t target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    int32_t start_l = Motor_GetTicks(motor_l);
    int32_t start_r = Motor_GetTicks(motor_r);
    int32_t average_dist = 0;
    int16_t commanded_speed = (inches < 0.0f) ? (int16_t)(-abs(speed)) : (int16_t)abs(speed);

    Motor_SetSpeed(motor_l, commanded_speed);
    Motor_SetSpeed(motor_r, commanded_speed);

    while (average_dist < target_ticks) {
        int32_t curr_l = labs(Motor_GetTicks(motor_l) - start_l);
        int32_t curr_r = labs(Motor_GetTicks(motor_r) - start_r);
        average_dist = (curr_l + curr_r) / 2;
    }

    Motor_Stop(motor_l);
    Motor_Stop(motor_r);
}

void Robot_Rotate(Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float degrees)
{
    int32_t target_ticks = (int32_t)(fabsf(degrees) * TICKS_PER_DEGREE);
    int32_t start_l = Motor_GetTicks(motor_l);
    int32_t start_r = Motor_GetTicks(motor_r);
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
        int32_t curr_l = labs(Motor_GetTicks(motor_l) - start_l);
        int32_t curr_r = labs(Motor_GetTicks(motor_r) - start_r);
        average_dist = (curr_l + curr_r) / 2;
    }

    Motor_Stop(motor_l);
    Motor_Stop(motor_r);
}