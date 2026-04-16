/**
  ******************************************************************************
  * @file    motor.h
  * @brief   This file contains all the function prototypes for
  *          the robot DC motors
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

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "constants.h"

#define PWM_PERIOD 1000

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t ch_fwd;
    uint32_t ch_bwd;
    uint32_t ch_en;
    TIM_HandleTypeDef* enc_timer;
    int32_t last_position;
} Motor_t;

void Motor_Init(Motor_t* motor);

void Motor_SetSpeed(Motor_t* motor, uint16_t speed);
void Motor_Stop(Motor_t* motor);

void Motor_ResetDistance(Motor_t* motor);
int32_t Motor_GetTicks(Motor_t* motor);

void Robot_MoveDistance(Motor_t* motor_l, Motor_t* motor_r, uint16_t speed, float inches);
void Robot_Rotate(Motor_t* motor_l, Motor_t* motor_r, uint16_t speed, float degrees);

#endif /*__ MOTOR_H__ */