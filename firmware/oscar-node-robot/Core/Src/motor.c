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

void Motor_Init(Motor_t* motor);
{
    HAL_TIM_PWM_Start(motor->htim, motor->ch_fwd);
    HAL_TIM_PWM_Start(motor->htim, motor->ch_bwd);
    HAL_TIM_PWM_Start(motor->htim, motor->ch_en);
}

void Motor_SetSpeed(Motor_t* motor, uint16_t speed)
{
    if (speed == 0) return;
    else if (speed > 0) {
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

void Robot_MoveDistance(Motor_t* motor_l, Motor_t* motor_r, uint16_t speed, float inches)
{
    int32_t target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    int32_t start_l = (int32_t)__HAL_TIM_GET_COUNTER(motor_l->enc_timer);
    int32_t start_r = (int32_t)__HAL_TIM_GET_COUNTER(motor_r->enc_timer);
    int32_t average_dist = 0;

    Motor_SetSpeed(motor_l, speed);
    Motor_SetSpeed(motor_r, speed);

    while (average_dist < target_ticks) {
        int32_t curr_l = abs((int32_t)__HAL_TIM_GET_COUNTER(left_motor->enc_timer) - start_l);
        int32_t curr_r = abs((int32_t)__HAL_TIM_GET_COUNTER(right_motor->enc_timer) - start_r);
        
        average_dist = (curr_l + curr_r) / 2;
    }

    Motor_Stop(Motor_t *motor);
}
void Robot_Rotate(Motor_t* motor_l, Motor_t* motor_r, uint16_t speed, uint16_t degrees)
{
    int32_t target_ticks = (int32_t)(fabsf(inches) * TICKS_PER_INCH);
    int32_t start_l = (int32_t)__HAL_TIM_GET_COUNTER(motor_l->enc_timer);
    int32_t start_r = (int32_t)__HAL_TIM_GET_COUNTER(motor_r->enc_timer);
    int32_t average_dist = 0;

    if (degrees == 0) return;
    else if (degress > 0) {
        // Rotate CCW
        Motor_SetSpeed(motor_l, -speed);
        Motor_SetSpeed(motor_r, speed);
    } else {
        // Rotate CW
        Motor_SetSpeed(motor_l, speed);
        Motor_SetSpeed(motor_r, -speed);
    }
    
    while (average_dist < target_ticks) {
        int32_t curr_l = abs((int32_t)__HAL_TIM_GET_COUNTER(left_motor->enc_timer) - start_l);
        int32_t curr_r = abs((int32_t)__HAL_TIM_GET_COUNTER(right_motor->enc_timer) - start_r);
        
        average_dist = (curr_l + curr_r) / 2;
    }

    Motor_Stop(Motor_t *motor);
}