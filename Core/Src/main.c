/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

osThreadId TempTaskHandle;
osThreadId adxlTaskHandle;
osThreadId uartTaskHandle;
osThreadId potTaskHandle;
osThreadId buzzTaskHandle;
xQueueHandle tempHandle;
xQueueHandle adxlHandle;
xQueueHandle adclHandle;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartTempTask(void const * argument);
void StartAdxlTask(void const * argument);
void StartUartTask(void const * argument);
void StartBuzzTask(void const * argument);
void StartAdcTask(void const * argument);

/* USER CODE BEGIN PFP */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ADXL345_Data;
uint8_t temp_data[2] = { 0, 0 };
int16_t read_lm75(void) {
	uint8_t LM75_ADDR =0x48;
    uint8_t reg = 0x00; // Temperature register
    HAL_I2C_Master_Transmit(&hi2c1, (LM75_ADDR << 1), &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, (LM75_ADDR << 1) | 0x01, temp_data, 2, HAL_MAX_DELAY);

    // Combine MSByte and LSByte and shift for 11-bit data
    int16_t temp_raw = (int16_t)(((temp_data[0] << 8) | temp_data[1]) >> 5);

    // Convert to deci-degrees Celsius (x10)
    return (temp_raw * 125) / 100;
}
int detect_fall(ADXL345_Data *accel) {
    // Convert raw values to g-force (assuming +/-2g and 10-bit resolution: 1 LSB ≈ 4mg)
    float x_g = accel->x * 0.004;
    float y_g = accel->y * 0.004;
    float z_g = accel->z * 0.004;
    char msg[50];ù
    char buf1[32];
    // Calculate the magnitude of acceleration vector
    float magnitude = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
    int scaled = (int)(magnitude * 100);  // Convert 0.00–0.99 → 0–99
   // sprintf(buf1, "Temp: %d C\r\n", scaled);


   	  	      // Send string via USART
   	  	     // HAL_UART_Transmit(&huart2, (uint8_t *)buf1, strlen(buf1), HAL_MAX_DELAY);
    // Free fall: magnitude ≈ 0g, Impact: magnitude > ~2g
    if (magnitude < 0.5f) {
        // Possible free fall detected
        osDelay(100); // Wait briefly and recheck to confirm impact
        return 1;
    }
    return 0;
}

int detect(ADXL345_Data *accel){
	 float x_g = accel->x * 0.004;
	    float y_g = accel->y * 0.004;
	    float z_g = accel->z * 0.004;
	    char msg[50];
	    char buf1[32];
	    float magnitude = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
	        int scaled = (int)(magnitude * 100);
	        return scaled;
}

void adxl345_init(void) {
    uint8_t data;
    uint16_t ADXL345_ADDR = 0x53;
    // Set POWER_CTL (0x2D) to Measure mode (bit 3 = 1)
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR << 1, 0x2D, 1, &data, 1, HAL_MAX_DELAY);

    // Set DATA_FORMAT (0x31) to full resolution, +/-2g (0x08)
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR << 1, 0x31, 1, &data, 1, HAL_MAX_DELAY);

    // Set BW_RATE (0x2C) to 100 Hz (0x0A)
    data = 0x0A;
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR << 1, 0x2C, 1, &data, 1, HAL_MAX_DELAY);
}
void adxl345_read(ADXL345_Data *accel) {
    uint8_t buffer[6];
    uint16_t ADXL345_ADDR = 0x53;
    // Read 6 bytes from DATAX0 (0x32) to DATAZ1 (0x37)
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR << 1, 0x32, 1, buffer, 6, HAL_MAX_DELAY);

    // Convert bytes to int16_t (little endian)
    accel->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    accel->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    accel->z = (int16_t)((buffer[5] << 8) | buffer[4]);
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

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
  /* definition and creation of temp */
  tempHandle =xQueueCreate(1,sizeof( uint16_t));


  /* definition and creation of adxl */
  adxlHandle =xQueueCreate(1,sizeof(ADXL345_Data));


  /* definition and creation of adc */
  adclHandle = xQueueCreate(1,sizeof(int32_t));
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TempTask */
  osThreadDef(TempTask, StartTempTask, osPriorityNormal, 0, 128);
    TempTaskHandle = osThreadCreate(osThread(TempTask), NULL);


  /* definition and creation of adxlTask */
    osThreadDef(adxlTask, StartAdxlTask, osPriorityNormal, 0, 128);
      adxlTaskHandle = osThreadCreate(osThread(adxlTask), NULL);

      /* definition and creation of uartTask */
      osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
      uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);
      /* definition and creation of buzzTask */
      osThreadDef(buzzTask, StartBuzzTask, osPriorityNormal, 0, 128);
      uartTaskHandle = osThreadCreate(osThread(buzzTask), NULL);
      /* definition and creation of adcTask */
           osThreadDef(adcTask, StartAdcTask, osPriorityNormal, 0, 128);
           uartTaskHandle = osThreadCreate(osThread(adcTask), NULL);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();

  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTempTask */
/**
  * @brief  Function implementing the TempTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTempTask */
void StartTempTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  int16_t temperature = read_lm75();
	  xQueueSendToBack(tempHandle,&temperature,0);
    osDelay(200);
  }
  /* USER CODE END 5 */
}


/* USER CODE BEGIN Header_StartAdxlTask */
/**
* @brief Function implementing the adxlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdxlTask */
void StartAdxlTask(void const * argument)
{
  /* USER CODE BEGIN StartAdxlTask */
  /* Infinite loop */
	ADXL345_Data accel;

	  adxl345_init();

  for(;;)
  {
	  adxl345_read(&accel);
	  	  xQueueSendToBack(adxlHandle,&accel,0);
	      osDelay(700);
	        // Check for fall
	       // if (detect_fall(&accel)) {
	        //	char msg[50];  // Make sure size > length of message
	        	//sprintf(msg, "FALL DETECTED!\r\n");
	        	//HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 0);

	        //}

	       // osDelay(200); // adjust sampling rate
	 // adxl345_read(&accel);
	  //xQueueSendToBack(adxlHandle,&accel,0);
    //osDelay(200);
  }
  /* USER CODE END StartAdxlTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
void StartUartTask(void const * argument)
{
	//ADXL345_Data accel;
	 int16_t temperature;
	 char buf1[32];
	 char buf2[64];
  /* Infinite loop */
  for(;;)
  {
xQueueReceive(tempHandle, &temperature, 0);
//xQueueReceive(adxlHandle,&accel,0);


	  	      // Convert temperature to string
	  	      // temperature / 10 = whole °C
	  	      // temperature % 10 = decimal part (deci-degrees)
	  	      sprintf(buf1, "Temp: %d.%d C\r\n", temperature / 10, abs(temperature % 10));

	  	      // Send string via USART
	  	      HAL_UART_Transmit(&huart2, (uint8_t *)buf1, strlen(buf1), HAL_MAX_DELAY);

	  	    	     // sprintf(buf2, "X: %d Y: %d Z: %d\r\n", accel.x, accel.y, accel.z);
	  	    	      //HAL_UART_Transmit(&huart2, (uint8_t *)buf2, strlen(buf2), HAL_MAX_DELAY);

	  	    	      HAL_Delay(500);


  }
}

/* USER CODE END Header_StartUartTask */

/* USER CODE BEGIN Header_StartbuzzTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
void StartBuzzTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	 int32_t mapped_value;
	 int16_t temperature;
	 ADXL345_Data accel;
	 uint8_t a=0;
	 char msg[50];
	 char buf1[50];
  /* Infinite loop */
  for(;;)
  {
	  if(a==0){

		  a=1;
		  osDelay(200);
	  }
	  xQueueReceive(adxlHandle,&accel,0);
	  xQueueReceive(adclHandle,&mapped_value,0);
	  xQueueReceive(tempHandle,&temperature,0);
	 int b = detect(&accel);
	 ///sprintf(buf1, " detect %d",b);

	   	  	      // Send string via USART
	   	  	     // HAL_UART_Transmit(&huart2, (uint8_t *)buf1, strlen(buf1), HAL_MAX_DELAY);
	   	  	 //osDelay(500);
	   	  	 if(temperature/10>mapped_value){
	   	  	 while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)){
	   	  		   	  		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	   	  		   	  	// sprintf(buf1, "%d", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
	   	  	  	  	      //HAL_UART_Transmit(&huart2, (uint8_t *)buf1, strlen(buf1), HAL_MAX_DELAY);

	   	  		   	  			    osDelay(200);

	   	  		   	  		 }
	   	  		   	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	   	  	 }
	   	  	 if(b<60){
	   	  		 while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)){
	   	  		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	   	  	 //sprintf(buf1, "%d", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
  	  	      //HAL_UART_Transmit(&huart2, (uint8_t *)buf1, strlen(buf1), HAL_MAX_DELAY);

	   	  			    osDelay(500);

	   	  		 }
	   	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	   	  	 }
/*if(temperature / 10>mapped_value ){
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	    osDelay(500);
}else{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
		    osDelay(500);
}
if(detect_fall(&accel)){
	  // Make sure size > length of message
	while(1){
		        	sprintf(msg, "FALL DETECTED!\r\n");
		        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 0);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
		    osDelay(500);
}
}*/


  }
  /* USER CODE END 5 */
}
void StartAdcTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	char msg[50];
	 char buf1[32];
	uint16_t adc_value;
	 int32_t mapped_value;
  for(;;)
  {

	  	  	  HAL_ADC_Start(&hadc1);
	  	     HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	     adc_value = HAL_ADC_GetValue(&hadc1); // 0–1023 for 10-bit ADC


	  	    mapped_value = (int32_t)((((float)adc_value / 1023) * 125.0) - 55.0);
	  	     sprintf(msg, "\n ADC = %4d", mapped_value);
	  	     HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  	     xQueueSend(adclHandle,&mapped_value,0);
	  	     osDelay(500);
  }
  /* USER CODE END 5 */
}
/* USER CODE BEGIN Header_StartbuzzTask */
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
