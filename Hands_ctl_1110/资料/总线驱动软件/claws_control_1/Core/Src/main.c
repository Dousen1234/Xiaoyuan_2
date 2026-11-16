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
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"
#include "claw_can_config.h"
#include "claw_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t led_at, position_at, status_at, diagnostic_at;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  Board_EarlySafe();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_RCC_EnableCSS();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  ClawControl_Init();
  ClawCanConfig_Init();
  Board_Init();
  led_at = position_at = status_at = diagnostic_at = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        if (Board_TakeFaults()) ClawControl_TransportFault();
        can_frame_t frame;
        uint8_t data[8];
        for (unsigned budget=0; budget<8u && Board_CAN_Pop(&frame); ++budget) {
            if (Board_TakeFaults()) { ClawControl_TransportFault(); break; }
            ClawCommand command;
            if (frame.id != CLAW_CAN_ID_COMMAND || frame.extended || frame.remote ||
                frame.dlc != 8 || HAL_GetTick() - frame.timestamp > APP_MAX_RX_AGE_MS ||
                !ClawProtocol_DecodeCommand(frame.data, &command)) continue;
            (void)ClawControl_HandleCommand(&command);
            if (command.opcode == CLAW_COMMAND_PARAM_SET || command.opcode == CLAW_COMMAND_PARAM_GET) {
                ClawControl_BuildParameterFrame(&command, data);
                (void)Board_Send(CLAW_CAN_ID_PARAMETER, data);
            }
            if (command.opcode != CLAW_COMMAND_HEARTBEAT) {
                ClawControl_BuildStatusFrame(data);
                (void)Board_Send(CLAW_CAN_ID_STATUS, data);
            }
        }
        ClawControl_Task();
        uint32_t now = HAL_GetTick();
        if (ClawControl_ConsumeNewFault()) {
            ClawControl_BuildStatusFrame(data); (void)Board_Send(CLAW_CAN_ID_FAULT, data);
        }
        if (now - position_at >= CLAW_POSITION_FEEDBACK_PERIOD_MS) {
            position_at = now;
            ClawControl_BuildPositionFrame(data); (void)Board_Send(CLAW_CAN_ID_POSITION, data);
        }
        if (now - status_at >= CLAW_STATUS_FEEDBACK_PERIOD_MS) {
            status_at = now;
            ClawControl_BuildStatusFrame(data); (void)Board_Send(CLAW_CAN_ID_STATUS, data);
        }
        if (now - diagnostic_at >= CLAW_DIAGNOSTIC_FEEDBACK_PERIOD_MS) {
            diagnostic_at = now;
            ClawControl_BuildDiagnosticFrame(data); (void)Board_Send(CLAW_CAN_ID_DIAGNOSTIC, data);
        }
        Board_CAN_Service();
        Board_FeedWatchdog();
        ClawStatus status; ClawControl_GetStatus(&status);
        if (now - led_at >= (status.fault_mask ? 100u : 500u)) {
            led_at = now; HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  Board_Panic();
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
