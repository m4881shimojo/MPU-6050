/* USER CODE BEGIN Header */
/**　Coded by M. shimojo 2022.06.12
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *MPU6050とMadgwickFilterを用�?た姿勢計測
  *Nucleo F446REをもち�?た�??
  *MPU6050 は事前にキャリブレーションを行い、加速度並びにジャイロセンサのオフセットを求めた
  *
  *拙いプログラムのため、ただの参考にしてください。間違いが多くあります。
  *This is a poor program and is just for reference only. There are many mistakes.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> //add shimojo
#include "TJ_MPU6050.h"
#include "MadgwickAHRS.h"
#include "MPU6050_Offset_Calibration.h"
#include "math.h"
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
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__IO uint8_t Pushed;  // Blue button Interrupt Flag (shimojo)

/* Gloval values */
const float PI=3.14159265358979;
extern volatile float beta; // add extern
extern volatile float q0, q1, q2, q3; // add extern
uint8_t data[5];
float Yaw_m,Pitch_m,Roll_m;

//It is made for sampling period measurement. Its value is observed by DEBUGGER(shimojo)
uint16_t timer_val; // timer start value
uint16_t timer_val2; // sampling period

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig; // ジャイロセンサの特性を決めるパラメータをset
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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  //printf("Gyro Start Madgwick3 \r\n");

  RawData_Def myAccelRaw, myGyroRaw; // add
  //ScaledData_Def myAccelScaled, myGyroScaled; // add

    int Ax,Ay,Az;//Raw data
    int Gx,Gy,Gz;//Raw data
    float Axf,Ayf,Azf;//Scaled data
    float Gxf,Gyf,Gzf;//Scaled data

    // Offset average data. Measured by shimojo.
    int Ax_bias,Ay_bias,Az_bias;//bias
    int Gx_bias,Gy_bias,Gz_bias;//bias
    Ax_bias=671;Ay_bias=-150;Az_bias=1099; // measured data from experiment by shimojo
    Gx_bias=-193;Gy_bias=86;Gz_bias=-50;// measured data from experiment by shimojo

    float AccelMetricScaling;
    //Here the gravity unit. Acceleration data will be normalized. (shimojo)
    AccelMetricScaling=(2000.0f/32768.0f);  //2000.0f for AFS_SEL_2g
    float GyroMetricScaling;
	// Gyro's Angular velocity must be radian. (shimojo)
    GyroMetricScaling= (250.0f/32768.0f)*(PI/180.0f);    //250.0f for FS_SEL_250
   // GyroMetricScaling= (500.0f/32768.0f);    //500.0f for FS_SEL_500


    /* The sampling frequency is calculated by the following formula
     * sampling frequency=1000/(5+delay(ms))
     * exsample 66.7=1000/(5+10)
     * CPU processing time=5ms, delay=10ms
     */
    MadgwickSetSampling(66.7f); // Set sampling Frequency (Added by shimojo)

    //timer is created for sampling period measurement. Its value is observed by DEBUGGER(shimojo)
     HAL_TIM_Base_Start(&htim14);     // Start timer

    //1. initialize the MPU6050 module and i2c
      MPU6050_Init(&hi2c1);

    //2.  configure MPU6050 (accel and gyro parameters)

      myMpuConfig.Accel_Full_Scale=AFS_SEL_2g  ;
      myMpuConfig.ClockSource = Internal_8MHz;
      //myMpuConfig.CONFIG_DLPF =  DLPF_21A_20G_Hz;// Delay 8.5ms
      myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;// Delay 2.0ms
     // myMpuConfig.CONFIG_DLPF = DLPF_260A_256G_Hz;//
      myMpuConfig.Gyro_Full_Scale =FS_SEL_250;
      //myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
      myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
      MPU6050_Config(&myMpuConfig);

      Ax_bias=671;Ay_bias=-150;Az_bias=1099; // measured data by experiment by shimojo
      Gx_bias=-193;Gy_bias=86;Gz_bias=-50;// measured data by experiment by shimojo

      //Here the gravity unit. Since acceleration is normalized. (shimojo)
      AccelMetricScaling=(2000.0f/32768.0f);  //2000.0f for AFS_SEL_2g

  	// Gyro's Angular velocity uses radians. (shimojo)
      GyroMetricScaling= (250.0f/32768.0f)*(PI/180.0f);    //250.0f for FS_SEL_500
   // GyroMetricScaling= (500.0f/32768.0f);    //500.0f for FS_SEL_500

     Pushed=1;//Flag: Blue button interrupt. if pressed Pushed become "1" (shimojo)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// Calculate offset value. Compensate only for Gyro offset data.
	//Because, Attitude drift is due to gyro angular velocity offset. (shimojo)
   //Caution: The current posture is not defined as the origin posture. (shimojo)
      if (Pushed==1)
      {
      MPU6050_Offset_Calibration(&Ax_bias, &Ay_bias, &Az_bias,  &Gx_bias, &Gy_bias, &Gz_bias );
      Pushed=0; //Flag: Blue button interrupt.
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Green LED Blink
      }

	//CPU processing interval measuament (shimojo)
      timer_val = __HAL_TIM_GET_COUNTER(&htim14);//Start timer

  //Get Raw data
	      MPU6050_Get_Accel_RawData(&myAccelRaw);//Get accel & Gyro Raw data
	      MPU6050_Get_Gyro_RawData(&myGyroRaw);// Pass Gyro Raw data to myGyroRaw
	      Ax=myAccelRaw.x-Ax_bias;  Ay=myAccelRaw.y-Ay_bias; Az=myAccelRaw.z-Az_bias;
	      Gx=myGyroRaw.x-Gx_bias;  Gy=myGyroRaw.y-Gy_bias; Gz=myGyroRaw.z-Gz_bias;

	      Axf=(float)Ax*(AccelMetricScaling);Ayf=(float)Ay*(AccelMetricScaling);Azf=(float)Az*(AccelMetricScaling);// gravity unit
	      Gxf=(float)Gx*(GyroMetricScaling);Gyf=(float)Gy*(GyroMetricScaling);Gzf=(float)Gz*(GyroMetricScaling);// radian/s
	      //Gxf=0.0f;Gyf=0.0f;Gzf=0.0f;// radian/s

	      /*STM32CubeIDEのprintfでfloatが使えな�?、対処方法�??*/
	      /*
	        プロジェクトを右click, properties,
	        C/C++ Build→Settings→MCU Settings,
	        Use float with printf from newlib-nano (-u _printf_float)にcheck
	       */

	      //Madgwick Filter.
	  		    MadgwickAHRSupdateIMU(Gxf,Gyf,Gzf,Axf,Ayf,Azf);

	  	HAL_Delay(10); //delay 10ms

	  	//Roll(X-axis),  Pitch(Y-axis), Yaw(Z-axis)   radian-->degree
	  	//pitchが90度付近でroll,Yawがおかしな挙動を示す？
	  	//Roll_m=(180.0f/PI)*atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);//degree
	  	//Pitch_m =(180.0f/PI)*asinf(-2.0f * (q1*q3 - q0*q2));
	  	//Yaw_m=(180.0f/PI)*atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	  	Roll_m=atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);//radian
	  	Pitch_m =asinf(-2.0f * (q1*q3 - q0*q2));
	  	Yaw_m=atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);

#if 1
	  	//Drawing graph by Processing

	  	printf ("%f",Roll_m); printf (",");//
		printf ("%f",Pitch_m); printf (",");//
	    printf ("%f",Yaw_m); printf ("  \r\n");//

  //Get  CPU elapsed processing time
	     timer_val2 = __HAL_TIM_GET_COUNTER(&htim14) - timer_val;
#endif

#if 0
		  	//MPU6050 measurd data quaternion
		  	printf ("%f",q0); printf (",");//
		  	printf ("%f",q1); printf (",");//
		  	printf ("%f",q2); printf (",");//
		  	printf ("%f",q3);  printf ("  \r\n");//
#endif

#if 0
		  	//MPU6050 measurd data
		  	printf ("%f",Axf); printf (",");//
		  	printf ("%f",Ayf); printf (",");//
		  	printf ("%f",Azf); printf (",");//
		  	printf ("%f",Gxf); printf (",");//
		  	printf ("%f",Gyf); printf (",");//
		  	printf ("%f",Gzf);  printf ("  \r\n");//
#endif

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 840-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //blueBottonの割込みルーチン
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		Pushed = 1;
	}
}


// printf用コー�?.ST-LinkよりTeratermに出力す�?,COM4経由かな
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
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
