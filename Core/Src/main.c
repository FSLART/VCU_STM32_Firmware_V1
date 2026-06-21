/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : CAN Test Isolation — stripped-down VCU
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stm32f767xx.h>
#include <stm32f7xx_hal_can.h>
#include <string.h>

/* ---- Test isolation flags ---- */
#define TEST_CAN_IRQ 1   /* 1 = use can_driver for interrupt-driven RX */
#define TEST_CAN_QUEUE 1 /* 1 = drain queues each loop, expose debug vars */
// #define TEST_DBC_CODEC   1   /* 1 = run DBC encode/decode demo once */
#define TEST_CAN_TX 1 /* 1 = send periodic test frames on all 3 buses */

#if TEST_CAN_IRQ
#include "can_driver.h"
#endif
#if TEST_CAN_QUEUE
#include "can_queue.h"
#endif
#if TEST_DBC_CODEC
#include "autonomous_temporary.h"
#include "data_dbc.h"
#include "dbc_codec.h"
#include "fsic.h"
#endif

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
/* Debug variables — watch these in debugger */
#if TEST_CAN_IRQ
/* can1/2/3_rx_count and can1/2/3_tx_count are defined in can_driver.c
   and declared extern in can_driver.h — watch them there. */
#endif

#if TEST_CAN_QUEUE
/* Snapshot variables updated each loop iteration */
uint32_t can1_queue_depth = 0;
uint32_t can2_queue_depth = 0;
uint32_t can3_queue_depth = 0;
can_msg_t last_can1_msg = {0};
can_msg_t last_can2_msg = {0};
can_msg_t last_can3_msg = {0};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- CAN RX interrupt callback ---- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
#if TEST_CAN_IRQ
    /* New path: route all three buses through can_driver */
    can_driver_rx_isr(hcan);
#else
    /* Legacy path: only handle CAN2 (powertrain) */
    if (hcan->Instance == CAN2) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            /* User: add legacy processing here if needed */
        }
    }
#endif
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if TEST_CAN_IRQ
    /* Init queues, filters, IRQs, start CAN buses */
    can_queue_init(&can1_rx_queue);
    can_queue_init(&can2_rx_queue);
    can_queue_init(&can3_rx_queue);
    can_driver_init();
    can_queue_init(&can1_tx_queue);
    can_queue_init(&can2_tx_queue);
    can_queue_init(&can3_tx_queue);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if TEST_CAN_IRQ
        /* Main-context queue drain — increment counters for debugger watch */
        can_msg_t msg;

        while (can_queue_pop(&can1_rx_queue, &msg)) {
            /* User: set breakpoint here and inspect msg.id, msg.data, msg.dlc */
            /* can1_rx_count is incremented in the ISR (can_driver_rx_isr) */
        }

        while (can_queue_pop(&can2_rx_queue, &msg)) {
            /* can2_rx_count is incremented in the ISR */
        }

        while (can_queue_pop(&can3_rx_queue, &msg)) {
            /* can3_rx_count is incremented in the ISR */
        }
#endif

#if TEST_CAN_QUEUE
        /* Snapshot queue depths for debugger watch */
        can1_queue_depth = can_queue_count(&can1_rx_queue);
        can2_queue_depth = can_queue_count(&can2_rx_queue);
        can3_queue_depth = can_queue_count(&can3_rx_queue);

        /* Peek at latest messages */
        can_queue_peek(&can1_rx_queue, &last_can1_msg);
        can_queue_peek(&can2_rx_queue, &last_can2_msg);
        can_queue_peek(&can3_rx_queue, &last_can3_msg);
#endif

#if TEST_DBC_CODEC
        /* Run once — encode + send a frame on each bus */
        static bool dbc_demo_done = false;
        if (!dbc_demo_done) {
            uint8_t data[8];
            uint8_t dlc;

            /* Powertrain (CAN2): encode + send INV1 SetERPM */
            if (dbc_encode_inv1_set_erpm(data, &dlc, 5000)) {
                can_driver_tx(&hcan2, FSIC_INV1_SET_ERPM_FRAME_ID, data, dlc);
            }

            /* Data bus (CAN1): encode + send VCU_0 */
            if (dbc_encode_vcu_0(data, &dlc, 0, 0, 1000, 800)) {
                can_driver_tx(&hcan1, DATA_DBC_VCU__FRAME_ID, data, dlc);
            }

            /* Autonomous bus (CAN3): encode + send VCU_RPM */
            if (dbc_encode_vcu_rpm(data, &dlc, 1200)) {
                can_driver_tx(&hcan3, AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID, data, dlc);
            }

            dbc_demo_done = true;
        }
#endif

#if TEST_CAN_TX
        /* Periodic test TX — 10ms rate, unique ID per bus, via TX queue */
        {
            static uint32_t last_tx_tick = 0;
            uint32_t now = HAL_GetTick();

            if ((now - last_tx_tick) >= 100) {
                last_tx_tick = now;

                /* Sequence counters — embedded in TX payload bytes [0..3] so the
                 * receiver can detect lost/reordered frames. Named _seq to avoid
                 * shadowing the hardware-level can1/2/3_tx_count in can_driver.c. */
                static uint32_t can1_tx_seq = 0;
                static uint32_t can2_tx_seq = 0;
                static uint32_t can3_tx_seq = 0;

                can_msg_t tx_msg;
                tx_msg.dlc = 8;
                tx_msg.timestamp = now;

                /* CAN3 (Autonomous): ID 0x303 */
                memset(tx_msg.data, 0, 8);
                tx_msg.id = 0x303;
                tx_msg.data[0] = (uint8_t)(can3_tx_seq & 0xFF);
                tx_msg.data[1] = (uint8_t)((can3_tx_seq >> 8) & 0xFF);
                tx_msg.data[2] = (uint8_t)((can3_tx_seq >> 16) & 0xFF);
                tx_msg.data[3] = (uint8_t)((can3_tx_seq >> 24) & 0xFF);
                tx_msg.bus = CAN_BUS_3;
                can_tx_enqueue(&tx_msg);
                can3_tx_seq++;

                /* CAN1 (Data bus): ID 0x301 */
                memset(tx_msg.data, 0, 8);
                tx_msg.id = 0x301;
                tx_msg.data[0] = (uint8_t)(can1_tx_seq & 0xFF);
                tx_msg.data[1] = (uint8_t)((can1_tx_seq >> 8) & 0xFF);
                tx_msg.data[2] = (uint8_t)((can1_tx_seq >> 16) & 0xFF);
                tx_msg.data[3] = (uint8_t)((can1_tx_seq >> 24) & 0xFF);
                tx_msg.bus = CAN_BUS_1;
                can_tx_enqueue(&tx_msg);
                can1_tx_seq++;

                /* CAN2 (Powertrain): ID 0x302 */
                memset(tx_msg.data, 0, 8);
                tx_msg.id = 0x302;
                tx_msg.data[0] = (uint8_t)(can2_tx_seq & 0xFF);
                tx_msg.data[1] = (uint8_t)((can2_tx_seq >> 8) & 0xFF);
                tx_msg.data[2] = (uint8_t)((can2_tx_seq >> 16) & 0xFF);
                tx_msg.data[3] = (uint8_t)((can2_tx_seq >> 24) & 0xFF);
                tx_msg.bus = CAN_BUS_2;
                can_tx_enqueue(&tx_msg);
                can2_tx_seq++;
            }
        }
#endif

        /* Drain TX queues — one attempt per bus per loop iteration */
        can_driver_tx_poll();

  /* USER CODE END 3 */
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
