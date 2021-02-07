/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "kalman.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "arm_math.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n)(*((volatile unsigned long *)(0xE0000000+4*n)))
#define DISPLAY_DBG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

kalman_state kstate;

int KalmanFilter(float *InputArray, float *OutputArray, kalman_state *kstate, int length){
	for(int i = 0; i < sizeof(InputArray); i++){
		if(InputArray[i] < 0){
			exit(0);
		}
		float measurement = InputArray[i];
		kalman(&kstate,measurement);
		OutputArray[i] = kstate->x;

	}
}

int kalman_c(float *InputArray, float *OutputArray, kalman_state *kstate, int length){
	int i = 0;
	while (i < length){
		kstate->p = kstate->p + kstate->q;
		kstate->k = kstate->p / (kstate->p + kstate->r);
		kstate->x = kstate->x + kstate->k * (InputArray[i] - kstate->x);
		if(kstate->x != kstate->x){
			return 1;
		}

		kstate->p = (1-kstate->k)*kstate->p;
		OutputArray[i] = kstate->x;
		i++;

	}
	return 0;

}

void sub_c(float *input1, float *input2, float *diff, int len){
	int i = 0;
	while(i <len){
		diff[i] = input1[i] - input2[i];
		i++;
	}
}


void avg_c(float *input,  int len, float *out){
	int i = 0;
	float temp = 0;
	while(i<len){
		temp += input[i];
		i++;
	}
	temp /= len;
	*out = temp;
}

void stdev_c(float *input, int len, float *result){
	float ss = 0;
	float sum = 0;
	int i = 0;
	while (i < len){
		ss += input[i]*input[i];
		sum += input[i];
		i++;
	}
	*result = sqrt((ss-(sum*sum)/len) / (len-1));
}



void conv_c(float *input1, float *input2, float *result, int len){
	int i,j;
	int newLen = 2*len -1;
	for(i=0; i<newLen; i++){
		result[i]=0.0;
		for(j=0;j<len;j++){
			int new = i-j;
			if(new<0){
				break;
			} else if (new<len){
				result[i] += input1[j] * input2[new];
			}
		}
	}
}

void corr_c(float *input1, float *input2, float *result, int len){
	int i,j;
	int newLen = 2*len -1;
	for(i=0; i< newLen; i++){
		result[i] = 0.0;
		for(j=0;j<len;j++){
			int new = len + j - i- 1;
			if(new< -len){
				break;
			} else if (new < len){
				result[i] += input1[j] * input2[new];
			}
		}
	}

	/*
	float mean1 = 0;
	float mean2 = 0;
	int i=0;
	while(i<len){
		mean1 += input1[i];
		i++;
	}
	mean1 = mean1/len;
	i=0;
	while(i<len){
			mean2 += input2[i];
			i++;
		}
	mean2 = mean2/len;
	float numerator = 0;
	float denominator = 0;
	float denominator1 = 0;
	float denominator2 = 0;
	i = 0;
	while(i<len){
		numerator = numerator + (input1[i]-mean1)*(input2[i]-mean2);
		i++;
	}
	i = 0;
	while(i<len){
		denominator1 = denominator1 + pow((input1[i]-mean1),2);
		i++;
	}
	i=0;
	while(i<len){
			denominator2 = denominator2 + pow((input2[i]-mean2),2);
			i++;
		}
	denominator = sqrt(denominator1*denominator2);
	*result = numerator / denominator;
	*result */
}







void kalman_cmsis(float *inputArr, float *cmsis_output, int len, kalman_state *ks){
	float sum_pr = 0;
	float sub_mx = 0;
	float mul_x = 0;
	float sub_k = 0;
	float a = 1;
	for(int i = 0; i< len; i++){
		//p=p+q
		arm_add_f32(&ks->p, &ks->q, &ks->p, 1);
		arm_add_f32(&ks->p, &ks->r, &sum_pr, 1);
		(ks->k) = (ks->p)/sum_pr;
		arm_sub_f32(&inputArr[i], &ks->x, &sub_mx, 1);
		arm_mult_f32(&ks->k, &sub_mx, &mul_x, 1);
		arm_add_f32(&ks->x, &mul_x, &ks->x, 1);
		arm_sub_f32(&a, &ks->k, &sub_k, 1);
		arm_mult_f32(&sub_k, &ks->q, &ks->p, 1);
		*(cmsis_output + i) = ks->x;
	}

}

float array_test[] = {8.36, 8.75, 10.625, 10.128735, 8.1927, 12.1286};


kalman_state cmsis;

int size = sizeof(array_test)/sizeof(float);



int main(void)
{
	/*
	// ---- TESTING ARM_LIB ----
	float array1[10];
	float array2[10];
	float32_t result[19];
	float32_t resultNum;
	arm_conv_f32(array1, 10, array2, 10, result);
	arm_std_f32(array1, 10, &resultNum);
	// ----
	 */


	//CMSIS Declarations
	float cmsis_output[size];
	float cmsis_diff[size];
	float cmsis_stdev;
	float cmsis_diffavg;
	float cmsis_corr[2*size-1];
	float cmsis_conv[2*size-1];

	float c_output[size];
	float c_diff[size];
	float c_stdev;
	float c_diffavg;
	float c_corr[2*size-1];
	float c_conv[2*size-1];

	// float *measurement;
	/*
	float q = 0.1;
	float r = 0.1;
	float x = 1.67;
	float p = 0.067;
	float k = 1.5;
	float measurement = 1.7;
	*/

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


  /* USER CODE END 2 */

  struct kalman_state c_state;
  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		float q = 0.1;
		float r = 0.1;
		float x = 5.0;
		float p = 0.1;
		float k = 0.0;
		float measurement;

		c_state.q = q;
		c_state.r = r;
		c_state.x = x;
		c_state.p = p;
		c_state.k = k;

		cmsis.q = q;
		cmsis.r = r;
		cmsis.x = x;
		cmsis.p = p;
		cmsis.k = k;

		//  ------------ CMSIS IMPLEMENTATIONS  ------------
		kalman_cmsis(array_test, cmsis_output, size, &cmsis);
		//1. Subtraction
		arm_sub_f32(array_test, cmsis_output, cmsis_diff, size);
		//2. Standard Deviation and Average of Differences
		arm_std_f32(cmsis_diff, size, &cmsis_stdev);
		arm_mean_f32(cmsis_diff, size, &cmsis_diffavg);
		//3. Correlation
		arm_correlate_f32(array_test, size, cmsis_output, size, cmsis_corr);
		//4. Convolution
		arm_conv_f32(array_test, size, cmsis_output, size, cmsis_conv);
		//  ------------ CMSIS IMPLEMENTATIONS  ------------

		//  ------------ C IMPLEMENTATIONS  ------------
		kalman_c(array_test, c_output, &c_state, size);

		sub_c(array_test, c_output, c_diff, size);
		stdev_c(c_diff, size, &c_stdev);
		avg_c(c_diff, size, &c_diffavg);
		conv_c(array_test, c_output, c_conv, size);
		corr_c(array_test, c_output, c_corr, size);

		//  ------------ C IMPLEMENTATIONS  ------------



/*
	  for(int i = 0; i<5; i++){
		  measurement = i;
		  kalman(&msg, measurement);
	  }
	  */

	 // kalman(&msg, &measurement);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
