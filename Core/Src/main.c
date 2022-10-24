/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Author: JackyPan
  *
  ******************************************************************************
**/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hal.h"
#include "event_groups.h"
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
MPU6000_Data mpu6000_data;
MPU9250_Data mpu9250_data;
ICM20608_Data icm20608_data;
QMC5883_Data qmc5883_data;
LSM303D_Data lsm303d_data;
L3GD20_Data l3gd20_data;
MS5607_Data ms5607_data;
MS5611_Data ms5611_data;
SPL06_Data spl06_data;
PWM_Channel pwm_channel;
uint8_t HeartBeatFlags=0;
uint32_t USB_Buffer_length=2048, URAT_DMA_Buffer_length=2048, RxBuffer_comm1_length, RxBuffer_comm2_length, RxBuffer_comm3_length, RxBuffer_comm4_length;
uint8_t *RxBuffer_comm1_DMA, *RxBuffer_comm2_DMA, *RxBuffer_comm3_DMA, *RxBuffer_comm4_DMA;
uint8_t *TxBuffer_comm0_buf, *TxBuffer_comm1_buf, *TxBuffer_comm2_buf, *TxBuffer_comm3_buf, *TxBuffer_comm4_buf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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

  /* USER CODE END 1 */

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM11_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  RxBuffer_comm1_DMA=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  RxBuffer_comm2_DMA=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  RxBuffer_comm3_DMA=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  RxBuffer_comm4_DMA=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  TxBuffer_comm0_buf=(uint8_t*)pvPortMalloc(USB_Buffer_length*sizeof(uint8_t));
  TxBuffer_comm1_buf=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  TxBuffer_comm2_buf=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  TxBuffer_comm3_buf=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  TxBuffer_comm4_buf=(uint8_t*)pvPortMalloc(URAT_DMA_Buffer_length*sizeof(uint8_t));
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim13);
  HAL_TIM_Base_Start_IT(&htim14);
  FATFS_UnLinkDriver(SDPath);
  Buzzer_Set_Output_Disable();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim11){// PPM
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1){
			PPM_RX_InterruptHandler();
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){
	case FMU_GPIO1_Pin:
		gpio1_interrupt_callback();
		break;
	case FMU_GPIO2_Pin:
		gpio2_interrupt_callback();
		break;
	case FMU_GPIO3_Pin:
		gpio3_interrupt_callback();
		break;
	case FMU_GPIO4_Pin:
		gpio4_interrupt_callback();
		break;
	case FMU_GPIO5_Pin:
		gpio5_interrupt_callback();
		break;
	case FMU_GPIO6_Pin:
		gpio6_interrupt_callback();
		break;
	case FMU_GPIO7_Pin:
		gpio7_interrupt_callback();
		break;
	case FMU_GPIO8_Pin:
		gpio8_interrupt_callback();
		break;
	default:
		break;
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7) {//400Hz
	  TIM_400HZ_Callback();
	  return;
  }

  if (htim->Instance == TIM10) {//200Hz
	  TIM_200HZ_Callback();
	  return;
   }

  if (htim->Instance == TIM13) {//100Hz
	  TIM_100HZ_Callback();
	  return;
  }

  if (htim->Instance == TIM14) {//50Hz
	  TIM_50HZ_Callback();
	  return;
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  usb_printf("error\n");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
