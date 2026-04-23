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
#include "bno055.h"

#define PWM_PERIOD 1000

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t ch_fwd;
    uint32_t ch_bwd;
    uint32_t ch_pwm;
    TIM_HandleTypeDef* enc_timer;
} Motor_t;

typedef enum {
    ROBOT_CTRL_OK = 0,
    ROBOT_CTRL_IMU_ERROR,
    ROBOT_CTRL_TIMEOUT,
    ROBOT_CTRL_PARAM_ERROR
} RobotControlStatus_t;

typedef struct {
    float kp;
    float kd;
    float corr_max;
    float heading_tolerance_deg;
    float rotate_kp;
    int16_t rotate_min_speed;
    uint32_t sample_period_ms;
    uint32_t timeout_ms;
} HeadingControlConfig_t;

void Motor_Init(Motor_t* motor);

void Motor_SetSpeed(Motor_t* motor, int16_t speed);
void Motor_Stop(Motor_t* motor);

void Motor_ResetDistance(Motor_t* motor);
int32_t Motor_GetTicks(Motor_t* motor);

void Robot_MoveDistance(Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float inches);
void Robot_Rotate(Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float degrees);
void HeadingControl_GetDefaultConfig(HeadingControlConfig_t* cfg);
RobotControlStatus_t Robot_MoveDistanceHeadingHold(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float inches);
RobotControlStatus_t Robot_RotateToDeltaHeading(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float delta_degrees);

uint8_t Robot_MotionRunner_IsBusy(void);
uint8_t Robot_MotionRunner_IsUsingImu(void);
void Robot_MotionRunner_Stop(void);
void Robot_MotionRunner_Tick(void);
RobotControlStatus_t Robot_MotionRunner_RequestMoveEnc(
    Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float inches);
RobotControlStatus_t Robot_MotionRunner_RequestRotateEnc(
    Motor_t* motor_l, Motor_t* motor_r, int16_t speed, float degrees);
RobotControlStatus_t Robot_MotionRunner_RequestMoveHeadingHold(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float inches);
RobotControlStatus_t Robot_MotionRunner_RequestRotateDeltaHeading(
    Motor_t* motor_l,
    Motor_t* motor_r,
    BNO055_t* imu,
    const HeadingControlConfig_t* cfg,
    int16_t speed,
    float delta_degrees);

#endif /*__ MOTOR_H__ */