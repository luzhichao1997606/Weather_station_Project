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
osThreadId APDS_TriggerHandle;
osThreadId DHT11_GetDataHandle;
osThreadId MPU6050_GetHandle;
osThreadId TFT_DisplayHandle;
/* USER CODE BEGIN PV */
double brightNess ;
QueueHandle_t Test_Queue = NULL;

SemaphoreHandle_t BinarySemaphore_Handle = NULL;
struct xLIST List_Test ;        //创建链表的根节点

struct xLIST_ITEM List_Item1;   //创建链表的节 
struct xLIST_ITEM List_Item2;   //创建链表的节 
struct xLIST_ITEM List_Item3;   //创建链表的节 

#define QUEUE_Len 		4 
#define QUEUE_Size		4 

uint8_t ID,MPU_State,DHT11_State ;

float pitch,roll,yaw;     //欧拉�?
short aacx,aacy,aacz;     //加�?�度传感器原始数�? 
short gyrox,gyroy,gyroz;	//�?螺仪原始数据
short temp;               //温度	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Task_Main_Start(void const * argument);
void LED_PWM_Task_Start(void const * argument);
void LED_Flash_Task_Start(void const * argument);
void RTC_GetTime_Start(void const * argument);
void APDS_GetData_Start(void const * argument);
void DHT11_DetData_Start(void const * argument);
void MPU6050_GetData_Start(void const * argument);
void TFT_DisplayAndShow(void const * argument);

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
	
	//链表初�?? 
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

  ID = APDS9960_Get_ID() ;
    if(ID == 0xab)
    { 
//      if (APDS9960_Init())
//      { 
//        APDS9960_Gesture_EN(1);
//      } 
    } 				
    //初始化MPU6050
//  while (MPU_Init()){ ; }
  
  temp = MPU_Get_Temperature();
  //等待初始化DMP单元
  //while(mpu_dmp_init())
 	//{ 
	//	;
	//}  

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */ 
  BinarySemaphore_Handle = xSemaphoreCreateBinary();
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

  /* definition and creation of APDS_Trigger */
  osThreadDef(APDS_Trigger, APDS_GetData_Start, osPriorityNormal, 0, 128);
  APDS_TriggerHandle = osThreadCreate(osThread(APDS_Trigger), NULL);

  /* definition and creation of DHT11_GetData */
  osThreadDef(DHT11_GetData, DHT11_DetData_Start, osPriorityNormal, 0, 128);
  DHT11_GetDataHandle = osThreadCreate(osThread(DHT11_GetData), NULL);

  /* definition and creation of MPU6050_Get */
  osThreadDef(MPU6050_Get, MPU6050_GetData_Start, osPriorityNormal, 0, 128);
  MPU6050_GetHandle = osThreadCreate(osThread(MPU6050_Get), NULL);

  /* definition and creation of TFT_Display */
  osThreadDef(TFT_Display, TFT_DisplayAndShow, osPriorityIdle, 0, 1024);
  TFT_DisplayHandle = osThreadCreate(osThread(TFT_Display), NULL);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_T_DO_GPIO_Port, TFT_T_DO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TFT_T_CS_Pin|TFT_T_CLK_Pin|TFT_DC_Pin|TFT_RESET_Pin 
                          |TFT_CS_Pin|TFT_SCK_Pin|TFT_SDO_Pin|LED_Pin 
                          |APDS_SCL_Pin|APDS_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_LED_GPIO_Port, TFT_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DHT11_SDA_Pin|RTC_SCL_Pin|RTC_SQW_Pin|RTC_32K_Pin 
                          |MPU6050_SCL_Pin|MPU6050_SDA_Pin|MPU6050_ADO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TFT_T_IRQ_Pin TFT_T_DIN_Pin */
  GPIO_InitStruct.Pin = TFT_T_IRQ_Pin|TFT_T_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_T_DO_Pin */
  GPIO_InitStruct.Pin = TFT_T_DO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_T_DO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_T_CS_Pin TFT_T_CLK_Pin TFT_DC_Pin TFT_RESET_Pin 
                           TFT_CS_Pin TFT_SCK_Pin TFT_SDO_Pin LED_Pin 
                           APDS_SCL_Pin APDS_SDA_Pin */
  GPIO_InitStruct.Pin = TFT_T_CS_Pin|TFT_T_CLK_Pin|TFT_DC_Pin|TFT_RESET_Pin 
                          |TFT_CS_Pin|TFT_SCK_Pin|TFT_SDO_Pin|LED_Pin 
                          |APDS_SCL_Pin|APDS_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_SDI_Pin */
  GPIO_InitStruct.Pin = TFT_SDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TFT_SDI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_LED_Pin DHT11_SDA_Pin RTC_SCL_Pin RTC_SQW_Pin 
                           RTC_32K_Pin MPU6050_SCL_Pin MPU6050_SDA_Pin MPU6050_ADO_Pin */
  GPIO_InitStruct.Pin = TFT_LED_Pin|DHT11_SDA_Pin|RTC_SCL_Pin|RTC_SQW_Pin 
                          |RTC_32K_Pin|MPU6050_SCL_Pin|MPU6050_SDA_Pin|MPU6050_ADO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EC11_B_Pin RTC_SDA_Pin MPU6050_INT_Pin */
  GPIO_InitStruct.Pin = EC11_B_Pin|RTC_SDA_Pin|MPU6050_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EC11_A_Pin EC11_KEY_Pin */
  GPIO_InitStruct.Pin = EC11_A_Pin|EC11_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : APDS_INT_Pin */
  GPIO_InitStruct.Pin = APDS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(APDS_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static  char    EC11_A_Last = 0;                        //EC11的A引脚上一次的状�??
static  char    EC11_B_Last = 0;                        //EC11的B引脚上一次的状�??
uint8_t         EC11_A_Now  = 0;                        //EC11的A引脚这一次的状�??
uint8_t         EC11_B_Now  = 0;                        //EC11的B引脚这一次的状�??
uint8_t GPIO_State ,State,Temp_Data,i;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	static BaseType_t xHigherPriorityTaskWoken;
  /* Prevent unused argument(s) compilation warning */
   
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	BaseType_t xReturn = pdPASS ;
  //APDS9960中断
  if(GPIO_Pin == APDS_INT_Pin)
  {
  if (APDS9960_Check_Gesture_State()) 
   {  
		  xSemaphoreGiveFromISR( BinarySemaphore_Handle, &xHigherPriorityTaskWoken );
   } 
  } 
  //EC11中断
  if (GPIO_Pin ==  EC11_A_Pin)
  { 
    EC11_A_Now = HAL_GPIO_ReadPin(EC11_A_GPIO_Port,EC11_A_Pin); 
     
    if(EC11_A_Now == 0)
      { 
        for(int i=0;i<750;i++); //延时消抖
        EC11_B_Now = HAL_GPIO_ReadPin(EC11_B_GPIO_Port,EC11_B_Pin); 
        if(EC11_B_Now ==1)      //只需要采集A的上升沿或下降沿的任意一个状态，              
          Temp_Data += 1;       //正转

        else                    //反转
          Temp_Data -= 1;
      }
      EC11_A_Last = EC11_A_Now;    
      EC11_B_Last = EC11_B_Now;    
    __HAL_GPIO_EXTI_CLEAR_IT(EC11_A_Pin); 
  }

  /* 如果 xHigherPriorityTaskWoken 表达式为 true ,要执行一次上下文切换*/
  	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  
	  
}


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
__weak void RTC_GetTime_Start(void const * argument)
{
  /* USER CODE BEGIN RTC_GetTime_Start */
  // uint8_t hour,sec,min ;
  /* Infinite loop */ 
  for(;;)
  { 
    
    //hour  =  DS3231_ReadDate.Hour   ;
    //min   =  DS3231_ReadDate.Minutes;
    //sec   =  DS3231_ReadDate.Seconds;
    //Read_RTC();
    vTaskSuspend(LED_PWM_TaskHandle);
    //vTaskSuspend(LED_Flash_TaskHandle);
    vTaskSuspend(DHT11_GetDataHandle); 
    vTaskSuspend(RTC_GetTimeHandle);  
    osDelay(1);
  }
  /* USER CODE END RTC_GetTime_Start */
}

/* USER CODE BEGIN Header_APDS_GetData_Start */
/**
* @brief Function implementing the APDS_Trigger thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APDS_GetData_Start */
__weak void APDS_GetData_Start(void const * argument)
{
  /* USER CODE BEGIN APDS_GetData_Start */
  /* Infinite loop */
  
  uint8_t ID,Dir,State,PIN_Set,Count_APDS_Run  ;  
  for(;;)
  { 
    Count_APDS_Run ++;
    if (Count_APDS_Run == 200)
    {
      Count_APDS_Run = 0;
    } 
		//获取二值信号量
		State = xSemaphoreTake(BinarySemaphore_Handle , 10);
		PIN_Set = HAL_GPIO_ReadPin(APDS_INT_GPIO_Port,APDS_INT_Pin); 
    if(ID == 0xab && State)
    {        
			APDS9960_Gesture_Get_State();//采用二值信号量
			
      if( !PIN_Set )
        { 
            decodeGesture();
            GPIO_State = 0;
            PIN_Set = 1;
            if (gesture_motion > 0)
            {
                switch (gesture_motion)
                {
                    case DIR_UP:
                        Dir = 0;
                        break;
                    case DIR_DOWN:
                        Dir = 1;
                        break;
                    case DIR_LEFT:
                        Dir = 2;
                        break;
                    case DIR_RIGHT:
                        Dir = 3;
                        break;
                    case DIR_NEAR:
                        Dir = 4;
                        break;
                    case DIR_FAR:
                        Dir = 5;
                        break;
                    default:
                        Dir = State;
                        break;
                }
            }
            resetGestureParameters(); 
        }
    }
		else if(!State)
    {
      //如果中断没有测到 
				
    } 
    else 
    {
       
      //vTaskSuspend(APDS_TriggerHandle);
    } 
      osDelay(1);
  }
  /* USER CODE END APDS_GetData_Start */
}

/* USER CODE BEGIN Header_DHT11_DetData_Start */
/**
* @brief Function implementing the DHT11_GetData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DHT11_DetData_Start */
__weak void DHT11_DetData_Start(void const * argument)
{
  /* USER CODE BEGIN DHT11_DetData_Start */
  /* Infinite loop */
  for(;;)
  {
    DHT11_State = DHT11_Init();
    //如果DHT11没有 测到则自挂东南枝
    if (DHT11_State)
    {
      vTaskSuspend(DHT11_GetDataHandle); 
    }
    
    osDelay(1);
  }
  /* USER CODE END DHT11_DetData_Start */
}

/* USER CODE BEGIN Header_MPU6050_GetData_Start */
/**
* @brief Function implementing the MPU6050_Get thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPU6050_GetData_Start */
__weak void MPU6050_GetData_Start(void const * argument)
{
  /* USER CODE BEGIN MPU6050_GetData_Start */
  /* Infinite loop */
  for(;;)
  {
    
			temp = MPU_Get_Temperature();	//得到温度 
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据 
    
    osDelay(1);
  }
  /* USER CODE END MPU6050_GetData_Start */
}

/* USER CODE BEGIN Header_TFT_DisplayAndShow */
/**
* @brief Function implementing the TFT_Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TFT_DisplayAndShow */
__weak void TFT_DisplayAndShow(void const * argument)
{
  /* USER CODE BEGIN TFT_DisplayAndShow */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TFT_DisplayAndShow */
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
