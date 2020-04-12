/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "list.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Count_PWM_Times = 0;
uint8_t Count_Flash_Times = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId Task_MainHandle;
osThreadId LED_PWM_TaskHandle;
osThreadId LED_Flash_TaskHandle;
osThreadId RTC_GetTimeHandle;
/* USER CODE BEGIN PV */
double brightNess ;
QueueHandle_t Test_Queue = NULL;
struct xLIST List_Test ;        //创建链表的根节点

struct xLIST_ITEM List_Item1;   //创建链表的节点
struct xLIST_ITEM List_Item2;   //创建链表的节 
struct xLIST_ITEM List_Item3;   //创建链表的节 

#define QUEUE_Len 		4 
#define QUEUE_Size		4 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Task_Main_Start(void const * argument);
void LED_PWM_Task_Start(void const * argument);
void LED_Flash_Task_Start(void const * argument);
void RTC_GetTime_Start(void const * argument);

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
  /* USER CODE BEGIN 2 */
  ModifyTime(0x11,0x11,0x11,0x11,0x11,0x11); 
  //链表根节点初始化
  vListInitialise( &List_Test );    
	
	//链表初始 
  vListInitialiseItem( & List_Item1 );
  List_Item1.xItemValue = 1;
	
	vListInitialiseItem( & List_Item2 );
  List_Item2.xItemValue = 2;
	
	vListInitialiseItem( & List_Item3 );
  List_Item3.xItemValue = 3;
	
	vListInsert( &List_Test , &List_Item1 );
	vListInsert( &List_Test , &List_Item2 );
	vListInsert( &List_Test , &List_Item3 );
	
	Test_Queue = xQueueCreate((UBaseType_t)QUEUE_Len,
														(UBaseType_t)QUEUE_Size);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_Main */
  osThreadDef(Task_Main, Task_Main_Start, osPriorityAboveNormal, 0, 128);
  Task_MainHandle = osThreadCreate(osThread(Task_Main), NULL);

  /* definition and creation of LED_PWM_Task */
  osThreadDef(LED_PWM_Task, LED_PWM_Task_Start, osPriorityIdle, 0, 128);
  LED_PWM_TaskHandle = osThreadCreate(osThread(LED_PWM_Task), NULL);

  /* definition and creation of LED_Flash_Task */
  osThreadDef(LED_Flash_Task, LED_Flash_Task_Start, osPriorityIdle, 0, 128);
  LED_Flash_TaskHandle = osThreadCreate(osThread(LED_Flash_Task), NULL);

  /* definition and creation of RTC_GetTime */
  osThreadDef(RTC_GetTime, RTC_GetTime_Start, osPriorityNormal, 0, 128);
  RTC_GetTimeHandle = osThreadCreate(osThread(RTC_GetTime), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RTC_SCL_Pin|RTC_SQW_Pin|RTC_32K_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RTC_SDA_Pin */
  GPIO_InitStruct.Pin = RTC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RTC_SCL_Pin RTC_SQW_Pin RTC_32K_Pin */
  GPIO_InitStruct.Pin = RTC_SCL_Pin|RTC_SQW_Pin|RTC_32K_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_Main_Start */
/**
  * @brief  Function implementing the Task_Main thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Task_Main_Start */
__weak void Task_Main_Start(void const * argument)
{ 
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    for (uint8_t i = 0; i < 100; i++)
    {
      osDelay(20);

      brightNess = sin(i * (3.14159265 / 100) );
    } 
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_LED_PWM_Task_Start */
/**
* @brief Function implementing the LED_PWM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_PWM_Task_Start */
__weak void LED_PWM_Task_Start(void const * argument)
{
  /* USER CODE BEGIN LED_PWM_Task_Start */
  /* Infinite loop */
	BaseType_t xReturn = pdTRUE;
	uint32_t T_Queue = 1;
  for(;;)
  {
    vTaskSuspend( LED_Flash_TaskHandle );
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    osDelay(brightNess * 20);
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    osDelay(20 - brightNess * 20);
		Count_PWM_Times ++ ;
		if(Count_PWM_Times == 150)
		{ 
			Count_Flash_Times = 1 ; 
			xReturn = xQueueSend(Test_Queue,&T_Queue,0);
      vTaskResume( LED_Flash_TaskHandle );
		}
		if(Count_PWM_Times == 255){
				Count_PWM_Times = 0;  
		}
  }
  /* USER CODE END LED_PWM_Task_Start */
}

/* USER CODE BEGIN Header_LED_Flash_Task_Start */
/**
* @brief Function implementing the LED_Flash_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Flash_Task_Start */
__weak void LED_Flash_Task_Start(void const * argument)
{
  /* USER CODE BEGIN LED_Flash_Task_Start */
  /* Infinite loop */
	BaseType_t xReturn = pdTRUE;
	uint32_t R_Queue;
  for(;;)
  {
    vTaskSuspend(LED_PWM_TaskHandle);
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
    osDelay(1000);
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
    osDelay(1000);
    if(Count_Flash_Times >= 1){
			Count_Flash_Times ++;
			if(Count_Flash_Times == 5){
				xReturn = xQueueReceive(Test_Queue,&R_Queue,portMAX_DELAY);
				if(pdTRUE == xReturn && R_Queue == 1){
					vTaskResume(LED_PWM_TaskHandle); 
				} 
			}
		}
  }
  /* USER CODE END LED_Flash_Task_Start */
}

/* USER CODE BEGIN Header_RTC_GetTime_Start */
/**
* @brief Function implementing the RTC_GetTime thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTC_GetTime_Start */

uint8_t hour,sec,min ;

__weak void RTC_GetTime_Start(void const * argument)
{
  
  /* USER CODE BEGIN RTC_GetTime_Start */
  /* Infinite loop */ 
  for(;;)
  { 
    
    hour = DS3231_ReadDate.Hour;
    min = DS3231_ReadDate.Minutes;
    sec =DS3231_ReadDate.Seconds;  
    Read_RTC();
    osDelay(1);
  }
  /* USER CODE END RTC_GetTime_Start */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
