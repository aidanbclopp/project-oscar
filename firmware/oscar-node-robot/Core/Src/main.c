/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "cc1101.h"
#include "log_runtime.h"
#include "motor.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define htim_mot_l htim2
#define htim_mot_r htim3
#define htim_enc_l htim4
#define htim_enc_r htim1
#define IMU_CALIB_TIMEOUT_MS 10000U
#define IMU_CALIB_SAMPLE_MS  100U
#define IMU_GYRO_ONLY_WAIT_MS 3000U
#define IMU_CALIB_MIN_SYS    2U
#define IMU_CALIB_MIN_GYRO   3U
#define IMU_CALIB_MIN_ACCEL  2U
#define IMU_CALIB_MIN_MAG    2U
#define RADIO_DRIVE_SPEED            700
#define RADIO_TURN_SPEED             650
#define RADIO_CMD_HOLD_TIMEOUT_MS    200U
#define MAIN_LED_BLINK_PERIOD_MS     500U
#define MAIN_VIEW_REFRESH_MS         100U
#define RADIO_EDGE_QUEUE_SIZE        128U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static Motor_t motor_l = {
    .htim = &htim_mot_l, 
    .ch_fwd = TIM_CHANNEL_1,
    .ch_bwd = TIM_CHANNEL_2,
    .ch_en  = TIM_CHANNEL_3,
    .enc_timer = &htim_enc_l
};
static Motor_t motor_r = {
    .htim = &htim_mot_r, 
    .ch_fwd = TIM_CHANNEL_1,
    .ch_bwd = TIM_CHANNEL_2,
    .ch_en  = TIM_CHANNEL_3,
    .enc_timer = &htim_enc_r
};
static BNO055_t bno055;
static HeadingControlConfig_t heading_cfg;
static uint8_t imu_ready = 0;
volatile uint8_t execute_user_routine = 0;
static uint8_t imu_init_ok = 0;
static uint8_t radio_link_ready = 0;
static uint8_t radio_motion_active = 0;
static uint32_t radio_last_cmd_ms = 0U;
static uint32_t cpu_cycles_per_us = 0U;
typedef struct {
  uint32_t ts_us;
  uint8_t level;
} RadioEdge_t;
static volatile RadioEdge_t radio_edge_queue[RADIO_EDGE_QUEUE_SIZE];
static volatile uint16_t radio_edge_head = 0U;
static volatile uint16_t radio_edge_tail = 0U;
static volatile uint8_t radio_edge_overflow = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Main_SetImuStatusLed(uint8_t on)
{
  /* PA12 status LED is wired active-high on this board. */
  HAL_GPIO_WritePin(BNO055_ON_GPIO_Port, BNO055_ON_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Main_UpdateImuReady(void)
{
  uint8_t sys_calib = 0U;
  uint8_t gyro_calib = 0U;

  if (!imu_init_ok) {
    imu_ready = 0U;
    Main_SetImuStatusLed(0U);
    return;
  }

  if (BNO055_ReadCalibStatus(&bno055, &sys_calib, &gyro_calib, NULL, NULL) != BNO055_OK) {
    imu_ready = 0U;
    Main_SetImuStatusLed(0U);
    printf("[MAIN] imu calib read failed\r\n");
    return;
  }

  imu_ready = (gyro_calib >= BNO055_MIN_GYRO_CALIB) ? 1U : 0U;
  Main_SetImuStatusLed(imu_ready);
  printf("[MAIN] imu calib sys=%u gyro=%u minGyroCal=%u imu_ready=%u\r\n",
         (unsigned int)sys_calib,
         (unsigned int)gyro_calib,
         (unsigned int)BNO055_MIN_GYRO_CALIB,
         (unsigned int)imu_ready);
}

static uint8_t Main_RunImuCalibrationSequence(void)
{
  uint32_t start_ms = HAL_GetTick();
  uint32_t last_hint_ms = 0U;
  uint8_t sys_calib = 0U;
  uint8_t gyro_calib = 0U;
  uint8_t accel_calib = 0U;
  uint8_t mag_calib = 0U;
  uint8_t led_state = 0U;

  if (!imu_init_ok) {
    imu_ready = 0U;
    Main_SetImuStatusLed(0U);
    printf("[MAIN] imu calibration skipped (imu init failed)\r\n");
    return 0U;
  }

  printf("[MAIN] imu calibration sequence start\r\n");
  while (1) {
    uint32_t elapsed_ms = HAL_GetTick() - start_ms;

    if (elapsed_ms >= IMU_CALIB_TIMEOUT_MS) {
      break;
    }

    if (BNO055_ReadCalibStatus(&bno055, &sys_calib, &gyro_calib, &accel_calib, &mag_calib) != BNO055_OK) {
      imu_ready = 0U;
      Main_SetImuStatusLed(0U);
      printf("[MAIN] imu calibration status read failed\r\n");
      return 0U;
    }

    printf("[MAIN] calib sys=%u gyro=%u accel=%u mag=%u\r\n",
           (unsigned int)sys_calib,
           (unsigned int)gyro_calib,
           (unsigned int)accel_calib,
           (unsigned int)mag_calib);

    if ((sys_calib >= IMU_CALIB_MIN_SYS) &&
        (gyro_calib >= IMU_CALIB_MIN_GYRO) &&
        (accel_calib >= IMU_CALIB_MIN_ACCEL) &&
        (mag_calib >= IMU_CALIB_MIN_MAG)) {
      imu_ready = 1U;
      Main_SetImuStatusLed(1U);
      printf("[MAIN] imu calibration ready\r\n");
      return 1U;
    }

    if ((elapsed_ms >= IMU_GYRO_ONLY_WAIT_MS) &&
        (gyro_calib >= IMU_CALIB_MIN_GYRO)) {
      imu_ready = 1U;
      Main_SetImuStatusLed(1U);
      printf("[MAIN] imu gyro-only ready (sys=%u accel=%u mag=%u)\r\n",
             (unsigned int)sys_calib,
             (unsigned int)accel_calib,
             (unsigned int)mag_calib);
      return 1U;
    }

    if ((elapsed_ms - last_hint_ms) >= 1000U) {
      printf("[MAIN] calibrating... keep robot still, then tilt/rotate slowly for accel/mag\r\n");
      last_hint_ms = elapsed_ms;
    }

    led_state ^= 1U;
    Main_SetImuStatusLed(led_state);
    HAL_Delay(IMU_CALIB_SAMPLE_MS);
  }

  imu_ready = 0U;
  Main_SetImuStatusLed(0U);
  printf("[MAIN] imu calibration timeout (using encoder-only)\r\n");
  return 0U;
}

static void Main_EnableCycleCounter(void)
{
  cpu_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t Main_GetMicros(void)
{
  if (cpu_cycles_per_us == 0U) {
    return HAL_GetTick() * 1000U;
  }
  return DWT->CYCCNT / cpu_cycles_per_us;
}

static void Main_ApplyRadioCommand(CC1101_Command_t cmd)
{
  if (cmd == CC1101_CMD_NONE) {
    Motor_Stop(&motor_l);
    Motor_Stop(&motor_r);
    radio_motion_active = 0U;
    return;
  }

  switch (cmd) {
    case CC1101_CMD_FORWARD:
      Motor_SetSpeed(&motor_l, RADIO_DRIVE_SPEED);
      Motor_SetSpeed(&motor_r, RADIO_DRIVE_SPEED);
      radio_motion_active = 1U;
      break;
    case CC1101_CMD_BACKWARD:
      Motor_SetSpeed(&motor_l, -RADIO_DRIVE_SPEED);
      Motor_SetSpeed(&motor_r, -RADIO_DRIVE_SPEED);
      radio_motion_active = 1U;
      break;
    case CC1101_CMD_LEFT:
      Motor_SetSpeed(&motor_l, -RADIO_TURN_SPEED);
      Motor_SetSpeed(&motor_r, RADIO_TURN_SPEED);
      radio_motion_active = 1U;
      break;
    case CC1101_CMD_RIGHT:
      Motor_SetSpeed(&motor_l, RADIO_TURN_SPEED);
      Motor_SetSpeed(&motor_r, -RADIO_TURN_SPEED);
      radio_motion_active = 1U;
      break;
    case CC1101_CMD_STOP:
    default:
      Motor_Stop(&motor_l);
      Motor_Stop(&motor_r);
      radio_motion_active = 0U;
      break;
  }
}

static void Main_ProcessRadioRx(void)
{
  if (!radio_link_ready) {
    return;
  }

  while (radio_edge_tail != radio_edge_head) {
    RadioEdge_t edge;
    CC1101_Command_t cmd;
    HAL_StatusTypeDef hal;
    uint32_t now_ms = HAL_GetTick();

    __disable_irq();
    edge = radio_edge_queue[radio_edge_tail];
    radio_edge_tail = (uint16_t)((radio_edge_tail + 1U) % RADIO_EDGE_QUEUE_SIZE);
    __enable_irq();

    cmd = CC1101_CMD_NONE;
    hal = CC1101_FeedEdge(edge.level, edge.ts_us, &cmd);
    if ((hal == HAL_OK) && (cmd != CC1101_CMD_NONE)) {
      Main_ApplyRadioCommand(cmd);
      radio_last_cmd_ms = now_ms;
      printf("[RADIO] cmd=%s\r\n", CC1101_CommandToString(cmd));
    }
  }

  if (radio_edge_overflow != 0U) {
    radio_edge_overflow = 0U;
    printf("[RADIO] edge queue overflow\r\n");
  }
}

static void Main_RadioSafetyStopCheck(uint32_t now_ms)
{
  if (!radio_motion_active) {
    return;
  }

  if ((now_ms - radio_last_cmd_ms) > RADIO_CMD_HOLD_TIMEOUT_MS) {
    Main_ApplyRadioCommand(CC1101_CMD_STOP);
    printf("[RADIO] hold timeout -> STOP\r\n");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Main_EnableCycleCounter();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LogRuntime_Init();
  LogRuntime_RefreshViews(16U);
  printf("[MAIN] persistent flash log initialized\r\n");

  BNO055_MountConfig_t mount_cfg;
  BNO055_GetDefaultMountConfig(&mount_cfg);
  mount_cfg.heading_axis = BNO055_AXIS_HEADING;
  mount_cfg.heading_sign = 1;
  mount_cfg.heading_offset_deg = 0.0f;

  imu_init_ok = (BNO055_Init(&bno055, &hi2c1, BNO055_I2C_ADDR_LOW, &mount_cfg) == BNO055_OK) ? 1U : 0U;
  Main_UpdateImuReady();
  printf("[MAIN] imu_init_ok=%u imu_ready=%u minGyroCal=%u\r\n",
         (unsigned int)imu_init_ok,
         (unsigned int)imu_ready,
         (unsigned int)BNO055_MIN_GYRO_CALIB);

  HeadingControl_GetDefaultConfig(&heading_cfg);
  Motor_Init(&motor_l);
  Motor_Init(&motor_r);
  printf("[MAIN] motor init complete\r\n");

  CC1101_AttachSpi(&hspi1);
  if (CC1101_InitForFlipperRemote() == HAL_OK) {
    if (CC1101_StartRx() == HAL_OK) {
      radio_link_ready = 1U;
      __HAL_GPIO_EXTI_CLEAR_IT(GDO0_Pin);
      HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
      HAL_NVIC_EnableIRQ(EXTI1_IRQn);
      printf("[RADIO] cc1101 ready in RX mode\r\n");
    } else {
      printf("[RADIO] failed to enter RX mode\r\n");
    }
  } else {
    printf("[RADIO] cc1101 init failed\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_led_ms = 0U;
    static uint32_t last_view_ms = 0U;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_led_ms) >= MAIN_LED_BLINK_PERIOD_MS) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      last_led_ms = now_ms;
    }

    if ((now_ms - last_view_ms) >= MAIN_VIEW_REFRESH_MS) {
      LogRuntime_RefreshViews(16U);
      last_view_ms = now_ms;
    }

    Main_ProcessRadioRx();
    Main_RadioSafetyStopCheck(now_ms);

    if (execute_user_routine && !radio_motion_active) {
      (void)Main_RunImuCalibrationSequence();
      Main_SetImuStatusLed(imu_ready);
      printf("[MAIN] user routine start imuMode=%s\r\n", imu_ready ? "heading-hold" : "encoder-only");
      // Move 2 feet, spin 180 CW, move 2 feet, spin 180 CCW.
      // Robot should end in same position and orientation as start.
      if (imu_ready) {
        (void)Robot_MoveDistanceHeadingHold(&motor_l, &motor_r, &bno055, &heading_cfg, 800, 24);
        (void)Robot_RotateToDeltaHeading(&motor_l, &motor_r, &bno055, &heading_cfg, 800, -180);
        (void)Robot_MoveDistanceHeadingHold(&motor_l, &motor_r, &bno055, &heading_cfg, 800, 24);
        (void)Robot_RotateToDeltaHeading(&motor_l, &motor_r, &bno055, &heading_cfg, 800, 180);
      } else {
        Robot_MoveDistance(&motor_l, &motor_r, 800, 24);
        Robot_Rotate(&motor_l, &motor_r, 800, -180);
        Robot_MoveDistance(&motor_l, &motor_r, 800, 24);
        Robot_Rotate(&motor_l, &motor_r, 800, 180);
      }
      execute_user_routine = 0;
      printf("[MAIN] user routine complete\r\n");
    }

    HAL_Delay(2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USER_BTN_Pin &&
        HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET) {
        execute_user_routine = 1;
        printf("[MAIN] user button pressed\r\n");
    } else if ((GPIO_Pin == GDO0_Pin) && (radio_link_ready != 0U)) {
        uint16_t next_head = (uint16_t)((radio_edge_head + 1U) % RADIO_EDGE_QUEUE_SIZE);
        uint8_t level = (HAL_GPIO_ReadPin(GDO0_GPIO_Port, GDO0_Pin) == GPIO_PIN_SET) ? 1U : 0U;
        uint32_t ts_us = Main_GetMicros();
        if (next_head == radio_edge_tail) {
            radio_edge_overflow = 1U;
            return;
        }
        radio_edge_queue[radio_edge_head].level = level;
        radio_edge_queue[radio_edge_head].ts_us = ts_us;
        radio_edge_head = next_head;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
