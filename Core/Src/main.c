/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery_lcd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT 0.01         // [s/loop] loop period. 20ms
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId Task01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;
osThreadId Task05Handle;
osMessageQId Queue01Handle;
osMessageQId Queue02Handle;
osMessageQId Queue03Handle;
/* USER CODE BEGIN PV */
float val[3];
char LCD_send[30];
char USB_send[30];
float rate_gyr_y = 0.0;   // [deg/s]
float rate_gyr_x = 0.0;   // [deg/s]
float rate_gyr_z = 0.0;   // [deg/s]
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float tmpX = 0.0;
float tmpY = 0.0;
float tmpY1 = 0.0;
uint16_t high = 50;
uint16_t RX = 120;
uint16_t RY = 170;
uint16_t x = 5;
uint16_t flag = 0;
uint16_t ponk = 0;
uint16_t score = 0;
uint16_t loose = 0;
uint16_t h = 0;
uint16_t inc = 1;
uint16_t mode = 1;
typedef struct
{
  float x_ang_rate;
  float y_ang_rate;
  float z_ang_rate;

} GYRO_DATA_T;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

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
  BSP_LCD_Init();//init LCD
  //set the layer buffer address into SDRAM
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);//select on which layer we write
  BSP_LCD_DisplayOn();//turn on LCD
  BSP_LCD_Clear(LCD_COLOR_DARKGREEN);//clear the LCD on blue color
  BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);//set text background color
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);//set text color
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

  /* Create the queue(s) */
  /* definition and creation of Queue01 */
  osMessageQDef(Queue01, 16, GYRO_DATA_T);
  Queue01Handle = osMessageCreate(osMessageQ(Queue01), NULL);

  /* definition and creation of Queue02 */
  osMessageQDef(Queue02, 16, uint16_t);
  Queue02Handle = osMessageCreate(osMessageQ(Queue02), NULL);

  /* definition and creation of Queue03 */
  osMessageQDef(Queue03, 16, uint16_t);
  Queue03Handle = osMessageCreate(osMessageQ(Queue03), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task01 */
  osThreadDef(Task01, StartTask01, osPriorityNormal, 0, 128);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, StartTask02, osPriorityNormal, 0, 128);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, StartTask03, osPriorityNormal, 0, 128);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* definition and creation of Task04 */
  osThreadDef(Task04, StartTask04, osPriorityNormal, 0, 256);
  Task04Handle = osThreadCreate(osThread(Task04), NULL);

  /* definition and creation of Task05 */
  osThreadDef(Task05, StartTask05, osPriorityIdle, 0, 128);
  Task05Handle = osThreadCreate(osThread(Task05), NULL);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GYRO_Task(void)
{
	BSP_GYRO_Init();
	GYRO_DATA_T gyro_data;
	osSignalWait(0x1, osWaitForever); //Wait for signal. Check cmsis_os.c for API name.
	for (;;)
	  {
		BSP_GYRO_GetXYZ(val);

		rate_gyr_x = val[0]  * G_GAIN;
		rate_gyr_y = val[1]  * G_GAIN;
		rate_gyr_z = val[2]  * G_GAIN;
		//BSP_LCD_FillTriangle(120,95,145,10,50,50);
		gyro_data.x_ang_rate+=rate_gyr_x*DT;
		gyro_data.y_ang_rate+=rate_gyr_y*DT;
		gyro_data.z_ang_rate+=rate_gyr_z*DT;
		if (gyro_data.x_ang_rate > 360){
			gyro_data.x_ang_rate = 360;
		}
		tmpY = tmpX;
	    /*Send gyroscope data to USB*/
	    /*GYRO_DATA_T *gyro_tx;
	    gyro_tx->x_ang_rate = gyro_data.x_ang_rate;
	    gyro_tx->y_ang_rate = gyro_data.y_ang_rate;
	    gyro_tx->z_ang_rate = gyro_data.z_ang_rate;*/
		osMessagePut(Queue01Handle, (uint32_t)&gyro_data, 0); //Put data into mail queue.
	    osEvent event1 = osMessageGet(Queue02Handle, osWaitForever); //Get queue
	    h = event1.value.p;
	    if ((gyro_data.x_ang_rate-tmpX)>=10 && h<70 && h>50 && RX>110 && RX<140 && RY>160 && RY<190 && inc == 0){
	    	//osSignalSet(Task03Handle, 0x1);
	    	high = 60;
	    	RY = 170;
	    	RX = 120;
	    	if ((gyro_data.x_ang_rate-tmpX)>=50){
	    		mode = 1;
	    	}
	    	else{
	    		mode = 2;
	    	}
	    	ponk = 1;
	    	osDelay(100);
	    	ponk = 0;
	    	flag = 0;
	    	//flag = 2;
	    }
	    /*else if ((tmpY-gyro_data.y_ang_rate)>=10 && h<70 && h>50 && RX>110 && RX<140 && RY>160 && RY<190 && inc == 0){
			high = 60;
			RY = 170;
			RX = 120;
			mode = 3;
			ponk = 1;
			osDelay(100);
			ponk = 0;
			flag = 0;
		}
	    else if ((gyro_data.y_ang_rate-tmpY)>=20 && h<70 && h>50 && RX>110 && RX<140 && RY>160 && RY<190 && inc == 0){
	    	high = 60;
	    	RY = 170;
	    	RX = 120;
	    	mode = 3;
	    	ponk = 1;
	    	osDelay(100);
	    	ponk = 0;
	    	flag = 1;
	    }*/
	    else if (h == 30 || RY == 190 || RX == 140){
	    	loose = 1;
	    }
	    	  tmpX = gyro_data.x_ang_rate;
	    	  //tmpY = gyro_data.y_ang_rate;
	    	  //gyro_data.y_ang_rate = tmpX;
	    	  //for (int i = 0;i<1500000;i++);
	    	  osDelay(100);
	    	  gyro_data.x_ang_rate = 0;
	    	  //gyro_data.y_ang_rate = 0;
	  }
}

void SCORE_Task(void)
{
	for(;;){
	//osSignalWait(0x1, osWaitForever);
		if (ponk == 1){
			score = score + 1;
			osDelay(200);
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
			osDelay(200);
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		if (loose == 1)
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
	}
}

void LCD_Task(void)
{
	//LCD
	for (;;)
	{
		    if (mode == 1){
		    	BSP_LCD_Clear(LCD_COLOR_DARKCYAN);
		    	BSP_LCD_SetBackColor(LCD_COLOR_DARKCYAN); // xoa lcd
		    	sprintf (LCD_send,"score = %d,%d",score,h); // doi int thanh string
		    	BSP_LCD_DisplayStringAtLine(1,LCD_send); // in string o dong 1
		    	BSP_LCD_FillCircle(RX,RY,high); // ve hinh tron (x,y) R = high
		    	Mode1();
		    }
		    else if (mode == 2){
		    	BSP_LCD_Clear(LCD_COLOR_DARKCYAN);
		    	BSP_LCD_SetBackColor(LCD_COLOR_DARKCYAN);
		    	sprintf (LCD_send,"score = %d,%d",score,RY);
		    	BSP_LCD_DisplayStringAtLine(1,LCD_send);
		    	BSP_LCD_FillCircle(RX,RY,high);
		    	Mode2();
		    }
		    /*else if (mode == 3){
		    	BSP_LCD_Clear(LCD_COLOR_DARKCYAN);
		    	BSP_LCD_SetBackColor(LCD_COLOR_DARKCYAN);
		    	sprintf (LCD_send,"score = %d,%d",score,RX);
		    	BSP_LCD_DisplayStringAtLine(1,LCD_send);
		    	BSP_LCD_FillCircle(RX,RY,high);
		    	Mode3();
		    }*/
	}
}

void Mode1(void) // bong to dần
{
	if (flag == 0){
		high= high + x;
		inc = 1;
		if (high == 120){
			flag = 1;
		}
	}
	else if (flag == 1){
		high = high - x;
		inc = 0;
		if (high == 20){
			flag = 0;
		}
	}
	osMessagePut(Queue02Handle, high, 0);
	osDelay(100);
}

void Mode2(void) // bóng bay thẳng
{
	if (flag == 0){
		RY= RY - 10;
		inc = 1;
		if (RY == 80){
			flag = 1;
		}
	}
	else if (flag == 1){
		RY = RY + 10;
		inc = 0;
		if (RY == 220)	{
			flag = 0;
		}
	}
	osMessagePut(Queue02Handle, high, 0);
	osDelay(100);
}


/*void Mode4(void){
	if (flag == 0)
	{
		RX = RX + 10;
		inc = 1;
		if (RX == 180){
			flag = 1;
		}
	}
	else if (flag == 1){
		RX = RX + 10;
		inc = 0;
		if (RX == 140){
			flag = 0;
		}
	}
	osMessagePut(Queue02Handle, high, 0);
	osDelay(100);
}*/
void USB_Task(void)
{
	osSignalWait(0x1, 400);                        //Wait for USB host to configure port
	//BSP_LCD_FillTriangle(120,95,145,10,50,50);
	osSignalSet(Task02Handle, 0x1);                          //Set signal in Task 02
	for (;;)
	{
		//osEvent event = osMessageGet(Queue01Handle, osWaitForever); //Get queue
		//GYRO_DATA_T *gyro_rx = event.value.p;
		//sprintf(USB_send,"x = %f, y = %f, z = %f \n",gyro_rx->x_ang_rate,gyro_rx->y_ang_rate,gyro_rx->z_ang_rate);
		//sprintf(USB_send,"x = %f, tmpX = %f, high = %d\n",gyro_rx->x_ang_rate,tmpY,h);
		if (mode == 1){
			sprintf(USB_send,"high = %d\n",h);
		}
		else if (mode == 2){
			sprintf(USB_send,"high = %d\n",RY);
		}
		/*else if (mode == 3){
			sprintf(USB_send,"high = %d\n",RX);
		}*/
		CDC_Transmit_HS(USB_send,strlen(USB_send));
		osDelay(100);
	}
}

void GameOver(void)
{
	//osSignalWait(0x1, osWaitForever);
	//for(;;){
		//if (loose == 1)
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
	//}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	LCD_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	GYRO_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	SCORE_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	USB_Task();
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Task05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
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
  while (1)
  {
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
