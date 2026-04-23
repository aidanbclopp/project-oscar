#include "robot_app.h"
#include "robot_app_tuning.h"

#include "bno055.h"
#include "cc1101.h"
#include "gpio.h"
#include "i2c.h"
#include "log_runtime.h"
#include "main.h"
#include "motor.h"
#include "spi.h"
#include "tim.h"

#include <stdio.h>

typedef enum {
  USER_ROUTINE_IDLE = 0U,
  USER_ROUTINE_STEP_1 = 1U,
  USER_ROUTINE_STEP_2 = 2U,
  USER_ROUTINE_STEP_3 = 3U,
  USER_ROUTINE_STEP_4 = 4U,
  USER_ROUTINE_DONE = 5U
} UserRoutineStep_t;

typedef struct {
  uint32_t led_ms;
  uint32_t view_ms;
  uint32_t imu_poll_ms;
} AppPeriodicState_t;

typedef struct {
  uint32_t ts_us;
  uint8_t level;
} RadioEdge_t;

typedef enum {
  APP_CMD_ACTION_NONE = 0U,
  APP_CMD_ACTION_MOVE,
  APP_CMD_ACTION_TURN,
  APP_CMD_ACTION_STOP
} AppCommandAction_t;

typedef struct {
  CC1101_Command_t cmd;
  AppCommandAction_t action;
  float value;
} AppCommandBehavior_t;

static Motor_t motor_l = {
    .htim = &htim2,
    .ch_fwd = TIM_CHANNEL_1,
    .ch_bwd = TIM_CHANNEL_2,
    .ch_pwm = TIM_CHANNEL_3,
    .enc_timer = &htim4
};

static Motor_t motor_r = {
    .htim = &htim3,
    .ch_fwd = TIM_CHANNEL_1,
    .ch_bwd = TIM_CHANNEL_2,
    .ch_pwm = TIM_CHANNEL_3,
    .enc_timer = &htim1
};

static BNO055_t bno055;
static HeadingControlConfig_t heading_cfg;
static uint8_t imu_ready = 0U;
static uint8_t execute_user_routine = 0U;
static uint8_t imu_init_ok = 0U;
static uint8_t radio_link_ready = 0U;

static CC1101_Command_t radio_last_logged_cmd = CC1101_CMD_NONE;
static uint32_t radio_last_log_ms = 0U;
static uint32_t radio_last_overflow_log_ms = 0U;
static uint32_t radio_overflow_count = 0U;
static CC1101_Command_t radio_pending_cmd = CC1101_CMD_NONE;

static volatile RadioEdge_t radio_edge_queue[APP_RADIO_EDGE_QUEUE_SIZE];
static volatile uint16_t radio_edge_head = 0U;
static volatile uint16_t radio_edge_tail = 0U;
static volatile uint8_t radio_edge_overflow = 0U;
static volatile uint32_t radio_last_edge_ts_us = 0U;

static uint32_t cpu_cycles_per_us = 0U;
static UserRoutineStep_t user_routine_step = USER_ROUTINE_IDLE;
static AppPeriodicState_t periodic_state = {0U, 0U, 0U};

/* Remote command mapping used by the app control loop. */
static const AppCommandBehavior_t k_radio_command_behaviors[] = {
    {CC1101_CMD_FORWARD,  APP_CMD_ACTION_MOVE, APP_RADIO_COMMAND_MOVE_INCHES},
    {CC1101_CMD_BACKWARD, APP_CMD_ACTION_MOVE, -APP_RADIO_COMMAND_MOVE_INCHES},
    {CC1101_CMD_LEFT,     APP_CMD_ACTION_TURN, APP_RADIO_COMMAND_TURN_DEG},
    {CC1101_CMD_RIGHT,    APP_CMD_ACTION_TURN, -APP_RADIO_COMMAND_TURN_DEG},
    {CC1101_CMD_CENTER,   APP_CMD_ACTION_TURN, APP_RADIO_COMMAND_CENTER_DEG},
    {CC1101_CMD_STOP,     APP_CMD_ACTION_STOP, 0.0f}
};

static void App_SetImuStatusLed(uint8_t on)
{
  HAL_GPIO_WritePin(BNO055_ON_GPIO_Port, BNO055_ON_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_SetRadioStatusLed(uint8_t on)
{
  HAL_GPIO_WritePin(RADIO_STATUS_GPIO_Port, RADIO_STATUS_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_SetEncoderStatusLed(uint8_t on)
{
  HAL_GPIO_WritePin(ENC_STATUS_GPIO_Port, ENC_STATUS_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_UpdateControlModeIndicator(void)
{
  if (!Robot_MotionRunner_IsBusy()) {
    /* Idle: indicate whether heading-hold is available right now. */
    App_SetImuStatusLed(imu_ready);
    return;
  }

  /* Active motion: indicate actual control mode in use. */
  App_SetImuStatusLed(Robot_MotionRunner_IsUsingImu());
}

static float App_ApplyMoveDistanceScale(float inches)
{
  if (inches > 0.0f) {
    return inches * APP_MOVE_FORWARD_SCALE;
  }
  if (inches < 0.0f) {
    return inches * APP_MOVE_BACKWARD_SCALE;
  }
  return inches;
}

static RobotControlStatus_t App_RequestMove(float inches)
{
  float tuned_inches = App_ApplyMoveDistanceScale(inches);

  if (imu_ready) {
    return Robot_MotionRunner_RequestMoveHeadingHold(
        &motor_l, &motor_r, &bno055, &heading_cfg, APP_RADIO_COMMAND_SPEED, tuned_inches);
  }
  return Robot_MotionRunner_RequestMoveEnc(&motor_l, &motor_r, APP_RADIO_COMMAND_SPEED, tuned_inches);
}

static RobotControlStatus_t App_RequestTurn(float degrees)
{
  float tuned_degrees = degrees;

  if (degrees < 0.0f) {
    tuned_degrees *= APP_ROTATE_CW_DEG_SCALE;
  } else if (degrees > 0.0f) {
    tuned_degrees *= APP_ROTATE_CCW_DEG_SCALE;
  }

  if (imu_ready) {
    return Robot_MotionRunner_RequestRotateDeltaHeading(
        &motor_l, &motor_r, &bno055, &heading_cfg, APP_RADIO_COMMAND_SPEED, tuned_degrees);
  }
  return Robot_MotionRunner_RequestRotateEnc(&motor_l, &motor_r, APP_RADIO_COMMAND_SPEED, tuned_degrees);
}

static void App_UpdateEncoderStatusLed(void)
{
  static uint8_t encoder_led_init = 0U;
  static int32_t prev_ticks_l = 0;
  static int32_t prev_ticks_r = 0;
  static uint32_t tick_accum = 0U;
  int32_t curr_ticks_l;
  int32_t curr_ticks_r;
  int32_t delta_l;
  int32_t delta_r;
  uint32_t abs_delta_l;
  uint32_t abs_delta_r;

  curr_ticks_l = Motor_GetTicks(&motor_l);
  curr_ticks_r = Motor_GetTicks(&motor_r);

  if (!encoder_led_init) {
    prev_ticks_l = curr_ticks_l;
    prev_ticks_r = curr_ticks_r;
    encoder_led_init = 1U;
    App_SetEncoderStatusLed(0U);
    return;
  }

  delta_l = curr_ticks_l - prev_ticks_l;
  delta_r = curr_ticks_r - prev_ticks_r;
  prev_ticks_l = curr_ticks_l;
  prev_ticks_r = curr_ticks_r;

  abs_delta_l = (delta_l < 0) ? (uint32_t)(-delta_l) : (uint32_t)delta_l;
  abs_delta_r = (delta_r < 0) ? (uint32_t)(-delta_r) : (uint32_t)delta_r;
  tick_accum += abs_delta_l + abs_delta_r;

  while (tick_accum >= APP_ENC_LED_TOGGLE_TICKS) {
    HAL_GPIO_TogglePin(ENC_STATUS_GPIO_Port, ENC_STATUS_Pin);
    tick_accum -= APP_ENC_LED_TOGGLE_TICKS;
  }
}

static void App_RefreshImuReadyFromSensor(uint8_t verbose_log)
{
  uint8_t sys_calib = 0U;
  uint8_t gyro_calib = 0U;
  uint8_t accel_calib = 0U;
  uint8_t mag_calib = 0U;
  uint8_t was_ready = imu_ready;
  static uint32_t last_calib_progress_log_ms = 0U;

  if (!imu_init_ok) {
    imu_ready = 0U;
    App_SetImuStatusLed(0U);
    if (verbose_log) {
      printf("[MAIN] imu not initialized\r\n");
    }
    return;
  }

  if (BNO055_ReadCalibStatus(&bno055, &sys_calib, &gyro_calib, &accel_calib, &mag_calib) != BNO055_OK) {
    imu_ready = 0U;
    App_SetImuStatusLed(0U);
    if (verbose_log || was_ready) {
      printf("[MAIN] imu calib read failed\r\n");
    }
    return;
  }

  imu_ready = (gyro_calib >= BNO055_MIN_GYRO_CALIB) ? 1U : 0U;
  App_SetImuStatusLed(imu_ready);

  if (verbose_log) {
    printf("[MAIN] imu calib sys=%u gyro=%u accel=%u mag=%u minGyroCal=%u imu_ready=%u\r\n",
           (unsigned int)sys_calib,
           (unsigned int)gyro_calib,
           (unsigned int)accel_calib,
           (unsigned int)mag_calib,
           (unsigned int)BNO055_MIN_GYRO_CALIB,
           (unsigned int)imu_ready);
    last_calib_progress_log_ms = HAL_GetTick();
  } else if (!imu_ready) {
    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_calib_progress_log_ms) >= 1000U) {
      printf("[MAIN] imu calibrating progress sys=%u gyro=%u accel=%u mag=%u (need gyro >= %u)\r\n",
             (unsigned int)sys_calib,
             (unsigned int)gyro_calib,
             (unsigned int)accel_calib,
             (unsigned int)mag_calib,
             (unsigned int)BNO055_MIN_GYRO_CALIB);
      last_calib_progress_log_ms = now_ms;
    }
  } else if (imu_ready != was_ready) {
    printf("[MAIN] imu_ready -> %u (sys=%u gyro=%u min=%u)\r\n",
           (unsigned int)imu_ready,
           (unsigned int)sys_calib,
           (unsigned int)gyro_calib,
           (unsigned int)BNO055_MIN_GYRO_CALIB);
  }
}

static uint8_t App_RunImuStartupCalibration(void)
{
  uint32_t start_ms = HAL_GetTick();
  uint8_t sys_calib = 0U;
  uint8_t gyro_calib = 0U;
  uint8_t accel_calib = 0U;
  uint8_t mag_calib = 0U;

  if (!imu_init_ok) {
    imu_ready = 0U;
    App_SetImuStatusLed(0U);
    printf("[MAIN] imu startup calibration skipped (imu init failed)\r\n");
    return 0U;
  }

  printf("[MAIN] imu startup gyro calibration start (keep robot flat/still)\r\n");
  while (1) {
    uint32_t elapsed_ms = HAL_GetTick() - start_ms;

    if (elapsed_ms >= APP_IMU_STARTUP_GYRO_TIMEOUT_MS) {
      break;
    }

    if (BNO055_ReadCalibStatus(&bno055, &sys_calib, &gyro_calib, &accel_calib, &mag_calib) != BNO055_OK) {
      imu_ready = 0U;
      App_SetImuStatusLed(0U);
      printf("[MAIN] imu calibration status read failed\r\n");
      return 0U;
    }

    if (gyro_calib >= APP_IMU_STARTUP_MIN_GYRO_CALIB) {
      imu_ready = 1U;
      App_SetImuStatusLed(1U);
      printf("[MAIN] imu startup gyro ready (sys=%u accel=%u mag=%u)\r\n",
             (unsigned int)sys_calib,
             (unsigned int)accel_calib,
             (unsigned int)mag_calib);
      return 1U;
    }

    printf("[MAIN] startup calib t=%lus sys=%u gyro=%u accel=%u mag=%u\r\n",
           (unsigned long)(elapsed_ms / 1000U),
           (unsigned int)sys_calib,
           (unsigned int)gyro_calib,
           (unsigned int)accel_calib,
           (unsigned int)mag_calib);
    HAL_Delay(APP_IMU_CALIB_SAMPLE_MS);
  }

  imu_ready = 0U;
  App_SetImuStatusLed(0U);
  printf("[MAIN] imu startup gyro timeout (using encoder-only)\r\n");
  return 0U;
}

void RobotApp_EnableCycleCounter(void)
{
  cpu_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t App_GetMicros(void)
{
  return DWT->CYCCNT / cpu_cycles_per_us;
}

static const AppCommandBehavior_t* App_FindCommandBehavior(CC1101_Command_t cmd)
{
  uint32_t i;
  uint32_t count = (uint32_t)(sizeof(k_radio_command_behaviors) / sizeof(k_radio_command_behaviors[0]));

  for (i = 0U; i < count; i++) {
    if (k_radio_command_behaviors[i].cmd == cmd) {
      return &k_radio_command_behaviors[i];
    }
  }
  return NULL;
}

static void App_ExecuteRadioDiscrete(CC1101_Command_t cmd)
{
  RobotControlStatus_t st = ROBOT_CTRL_OK;
  const AppCommandBehavior_t* behavior = App_FindCommandBehavior(cmd);

  if (behavior == NULL) {
    return;
  }

  if (behavior->action == APP_CMD_ACTION_STOP) {
    radio_pending_cmd = CC1101_CMD_NONE;
    Robot_MotionRunner_Stop();
    Motor_Stop(&motor_l);
    Motor_Stop(&motor_r);
    return;
  }

  switch (behavior->action) {
    case APP_CMD_ACTION_MOVE:
      st = App_RequestMove(behavior->value);
      break;
    case APP_CMD_ACTION_TURN:
      st = App_RequestTurn(behavior->value);
      break;
    case APP_CMD_ACTION_NONE:
    case APP_CMD_ACTION_STOP:
    default:
      return;
  }

  if (st != ROBOT_CTRL_OK) {
    printf("[RADIO] motion request failed cmd=%s st=%d\r\n", CC1101_CommandToString(cmd), (int)st);
  }
}

static void App_FlushPendingRadioCmd(void)
{
  CC1101_Command_t c;

  if (radio_pending_cmd == CC1101_CMD_NONE) {
    return;
  }
  if (Robot_MotionRunner_IsBusy()) {
    return;
  }
  c = radio_pending_cmd;
  radio_pending_cmd = CC1101_CMD_NONE;
  App_ExecuteRadioDiscrete(c);
}

static uint8_t App_PopRadioEdge(RadioEdge_t *edge)
{
  if (radio_edge_tail == radio_edge_head) {
    return 0U;
  }

  __disable_irq();
  *edge = radio_edge_queue[radio_edge_tail];
  radio_edge_tail = (uint16_t)((radio_edge_tail + 1U) % APP_RADIO_EDGE_QUEUE_SIZE);
  __enable_irq();
  return 1U;
}

static void App_HandleDecodedRadioCmd(CC1101_Command_t cmd, uint32_t now_ms)
{
  if (Robot_MotionRunner_IsBusy() && (cmd != CC1101_CMD_STOP)) {
    radio_pending_cmd = cmd;
    return;
  }

  if ((cmd != radio_last_logged_cmd) || ((now_ms - radio_last_log_ms) >= APP_RADIO_CMD_LOG_PERIOD_MS)) {
    printf("[RADIO] cmd=%s\r\n", CC1101_CommandToString(cmd));
    radio_last_logged_cmd = cmd;
    radio_last_log_ms = now_ms;
  }

  App_ExecuteRadioDiscrete(cmd);
  radio_pending_cmd = CC1101_CMD_NONE;
}

static void App_HandleRadioOverflow(void)
{
  if (radio_edge_overflow == 0U) {
    return;
  }

  __disable_irq();
  radio_edge_tail = radio_edge_head;
  __enable_irq();
  radio_edge_overflow = 0U;
  radio_overflow_count++;
}

static void App_LogRadioOverflow(void)
{
  uint32_t now_ms = HAL_GetTick();

  if ((radio_overflow_count == 0U) ||
      ((now_ms - radio_last_overflow_log_ms) < APP_RADIO_OVERFLOW_LOG_PERIOD_MS)) {
    return;
  }

  printf("[RADIO] edge queue overflow x%lu\r\n", (unsigned long)radio_overflow_count);
  radio_overflow_count = 0U;
  radio_last_overflow_log_ms = now_ms;
}

static void App_ProcessRadioRx(void)
{
  uint32_t edges_this_call = 0U;
  RadioEdge_t edge;

  if (!radio_link_ready) {
    return;
  }

  while (App_PopRadioEdge(&edge) != 0U) {
    CC1101_Command_t cmd = CC1101_CMD_NONE;

    edges_this_call++;
    if ((edges_this_call % APP_RADIO_EDGE_TICK_EVERY) == 0U) {
      Robot_MotionRunner_Tick();
    }

    if (CC1101_FeedEdge(edge.level, edge.ts_us, &cmd) == HAL_OK && (cmd != CC1101_CMD_NONE)) {
      App_HandleDecodedRadioCmd(cmd, HAL_GetTick());
    }
  }

  App_HandleRadioOverflow();
  App_LogRadioOverflow();
}

static uint8_t App_IsPeriodElapsed(uint32_t now_ms, uint32_t *last_ms, uint32_t period_ms)
{
  if ((now_ms - *last_ms) < period_ms) {
    return 0U;
  }
  *last_ms = now_ms;
  return 1U;
}

static void App_RunPeriodicTasks(uint32_t now_ms)
{
  if (App_IsPeriodElapsed(now_ms, &periodic_state.imu_poll_ms, APP_IMU_POLL_MS)) {
    App_RefreshImuReadyFromSensor(0U);
  }

  if (App_IsPeriodElapsed(now_ms, &periodic_state.led_ms, APP_MAIN_LED_BLINK_PERIOD_MS)) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

  if (App_IsPeriodElapsed(now_ms, &periodic_state.view_ms, APP_VIEW_REFRESH_MS)) {
    LogRuntime_RefreshViews(16U);
  }
}

static void App_UpdateUserRoutine(void)
{
  if (!execute_user_routine) {
    user_routine_step = USER_ROUTINE_IDLE;
    return;
  }

  if (Robot_MotionRunner_IsBusy()) {
    return;
  }

  if (user_routine_step == USER_ROUTINE_IDLE) {
    printf("[MAIN] user routine start imuMode=%s\r\n", imu_ready ? "heading-hold" : "encoder-only");
    user_routine_step = USER_ROUTINE_STEP_1;
    return;
  }

  switch (user_routine_step) {
    case USER_ROUTINE_STEP_1:
    case USER_ROUTINE_STEP_3:
      (void)App_RequestMove(APP_USER_ROUTINE_MOVE_INCHES);
      break;
    case USER_ROUTINE_STEP_2:
      (void)App_RequestTurn(-APP_USER_ROUTINE_TURN_DEG);
      break;
    case USER_ROUTINE_STEP_4:
      (void)App_RequestTurn(APP_USER_ROUTINE_TURN_DEG);
      break;
    default:
      break;
  }

  user_routine_step++;
  if (user_routine_step == USER_ROUTINE_DONE) {
    execute_user_routine = 0U;
    user_routine_step = USER_ROUTINE_IDLE;
    printf("[MAIN] user routine complete\r\n");
  }
}

static void App_InitLogging(void)
{
  LogRuntime_Init();
  LogRuntime_RefreshViews(16U);
  printf("[MAIN] persistent flash log %s (OSCAR_ENABLE_PERSISTENT_LOGS=%u)\r\n",
         (OSCAR_ENABLE_PERSISTENT_LOGS != 0U) ? "enabled" : "disabled",
         (unsigned int)OSCAR_ENABLE_PERSISTENT_LOGS);
}

static void App_InitImu(void)
{
  BNO055_MountConfig_t mount_cfg;

  BNO055_GetDefaultMountConfig(&mount_cfg);
  mount_cfg.heading_axis = BNO055_AXIS_HEADING;
  mount_cfg.heading_sign = 1;
  mount_cfg.heading_offset_deg = 0.0f;

  imu_init_ok = (BNO055_Init(&bno055, &hi2c1, BNO055_I2C_ADDR_LOW, &mount_cfg) == BNO055_OK) ? 1U : 0U;
  App_RefreshImuReadyFromSensor(1U);
  (void)App_RunImuStartupCalibration();

  printf("[MAIN] imu_init_ok=%u imu_ready=%u minGyroCal=%u\r\n",
         (unsigned int)imu_init_ok,
         (unsigned int)imu_ready,
         (unsigned int)BNO055_MIN_GYRO_CALIB);
}

static void App_InitMotors(void)
{
  HeadingControl_GetDefaultConfig(&heading_cfg);
  heading_cfg.kp = APP_HEADING_KP;
  heading_cfg.kd = APP_HEADING_KD;
  heading_cfg.corr_max = APP_HEADING_CORR_MAX;
  heading_cfg.rotate_kp = APP_ROTATE_KP;
  heading_cfg.rotate_min_speed = APP_ROTATE_MIN_SPEED;
  heading_cfg.heading_tolerance_deg = APP_HEADING_TOL_DEG;
  heading_cfg.sample_period_ms = APP_HEADING_SAMPLE_MS;
  heading_cfg.timeout_ms = APP_HEADING_TIMEOUT_MS;

  Motor_Init(&motor_l);
  Motor_Init(&motor_r);
  App_SetEncoderStatusLed(0U);
  printf("[MAIN] motor init complete\r\n");
}

static void App_InitRadio(void)
{
  CC1101_AttachSpi(&hspi1);
  if (CC1101_InitForFlipperRemote() != HAL_OK) {
    App_SetRadioStatusLed(0U);
    printf("[RADIO] cc1101 init failed\r\n");
    return;
  }

  if (CC1101_StartRx() != HAL_OK) {
    App_SetRadioStatusLed(0U);
    printf("[RADIO] failed to enter RX mode\r\n");
    return;
  }

  radio_link_ready = 1U;
  App_SetRadioStatusLed(1U);
  __HAL_GPIO_EXTI_CLEAR_IT(GDO0_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  printf("[RADIO] cc1101 ready in RX mode\r\n");
}

void RobotApp_Init(void)
{
  App_InitLogging();
  App_InitImu();
  App_InitMotors();
  App_InitRadio();
}

void RobotApp_Tick(void)
{
  uint32_t now_ms = HAL_GetTick();

  App_ProcessRadioRx();
  Robot_MotionRunner_Tick();
  App_UpdateControlModeIndicator();
  App_FlushPendingRadioCmd();
  App_UpdateEncoderStatusLed();
  App_RunPeriodicTasks(now_ms);
  App_UpdateUserRoutine();
}

void RobotApp_OnGpioExti(uint16_t gpio_pin)
{
  if (gpio_pin == USER_BTN_Pin &&
      HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET) {
    execute_user_routine = 1U;
    printf("[MAIN] user button pressed\r\n");
    return;
  }

  if ((gpio_pin != GDO0_Pin) || (radio_link_ready == 0U)) {
    return;
  }

  {
    uint16_t next_head = (uint16_t)((radio_edge_head + 1U) % APP_RADIO_EDGE_QUEUE_SIZE);
    uint8_t level = (HAL_GPIO_ReadPin(GDO0_GPIO_Port, GDO0_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    uint32_t ts_us = App_GetMicros();
    uint32_t edge_dt_us = ts_us - radio_last_edge_ts_us;

    if ((radio_last_edge_ts_us != 0U) && (edge_dt_us < APP_RADIO_EDGE_MIN_DELTA_US)) {
      return;
    }
    radio_last_edge_ts_us = ts_us;

    if (next_head == radio_edge_tail) {
      radio_edge_tail = (uint16_t)((radio_edge_tail + 1U) % APP_RADIO_EDGE_QUEUE_SIZE);
      radio_edge_overflow = 1U;
      next_head = (uint16_t)((radio_edge_head + 1U) % APP_RADIO_EDGE_QUEUE_SIZE);
    }

    radio_edge_queue[radio_edge_head].level = level;
    radio_edge_queue[radio_edge_head].ts_us = ts_us;
    radio_edge_head = next_head;
  }
}
