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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define test 1
#define V_cons 15 //15V
#define ADC_BUFFER_SIZE 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t dma_flag = 0;
volatile uint8_t tim1_flag = 0;
// 电压控制功率
float V_value = 0;
float I_value = 0;
// 信号buffer
float ref_buffer[ADC_BUFFER_SIZE/4] = {0};
float out_buffer[ADC_BUFFER_SIZE/4] = {0};
int ref_trigger[10];
int out_trigger[10];
float ref_freq = 0;
float out_freq = 0;
float  V_sum = 0;
float I_sum = 0;
uint16_t j = 0;
uint16_t k = 0;
uint16_t ref_count;
uint16_t out_count;
//“零点�?�和系数
float sin_k = 0.5;
float trigger = 1.00;
//更新频率初始�????
uint16_t autoreload = 140;

uint16_t phase_diff = 0;

//adc buffer
uint16_t adc_buffer[ADC_BUFFER_SIZE]={0};

//sin �????????
int sin_table[128]={
0,24,49,73,97,121,145,168,191,213,235,257,277,297,317,335,
353,370,386,401,415,428,440,451,461,470,478,485,490,494,497,499,
500,499,497,494,490,485,478,470,461,451,440,428,415,401,386,370,
353,335,317,297,277,257,235,213,191,168,145,121,97,73,49,24,
0,-24,-49,-73,-97,-121,-145,-168,-191,-213,-235,-257,-277,-297,-317,-335,
-353,-370,-386,-401,-415,-428,-440,-451,-461,-470,-478,-485,-490,-494,-497,-499,
-500,-499,-497,-494,-490,-485,-478,-470,-461,-451,-440,-428,-415,-401,-386,-370,
-353,-335,-317,-297,-277,-257,-235,-213,-191,-168,-145,-121,-97,-73,-49,-24
};
uint16_t sin_index = 0;


/*Oled*/
uint8_t ref_f[] = "ref_f:";
char ref_f_act[50];
uint8_t out_f[] = "out_f:";
char out_f_act[50];
uint8_t aar[] = "AAR:";
char aar_act[50];
uint8_t V[] = "V:";
char V_act[50];
uint32_t oled_count = 0;

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //�????????启pwm�????????
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim3);
	
  //�????????启定时器中断兼dma
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);

  //�????????启载波定时器中断
  HAL_TIM_Base_Start_IT(&htim1);
  
  // 初始化OLED
	OLED_Init();
  OLED_Display_On();
  OLED_Clear();

  OLED_ShowString(10,0,ref_f,sizeof(ref_f));
	OLED_ShowString(10,2,out_f,sizeof(out_f));
	OLED_ShowString(10,4,aar,sizeof(aar));
  OLED_ShowString(10,6,V,sizeof(V));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //dma处理
    if(dma_flag == 1)
    {
			j=0;
			k=0;
      HAL_ADC_Stop_DMA(&hadc1);
      dma_flag = 0;
      for(int i = 0; i < ADC_BUFFER_SIZE; i++)
      {
        if(i%4 == 0)
        {
           V_sum += adc_buffer[i]/4096.0;
        }
        else if (i%4 == 1)
        {
          I_sum += adc_buffer[i]/4096.0;
        }
        else if(i%4 == 2)
        {
          ref_buffer[k++] = adc_buffer[i]*3.3/4096;
        }
        else if (i%4 == 3)
        {
					out_buffer[j++] = adc_buffer[i]*3.3/4096;
        }
      }

      // 计算电压控制功率
      V_value = V_sum*3.3/(ADC_BUFFER_SIZE/4);
      I_value = I_sum*3.3/(ADC_BUFFER_SIZE/4);

      snprintf(V_act, sizeof(V_act), "%.2fV",V_value);
      OLED_ShowString(80, 6, (uint8_t*)V_act, sizeof(V_act));
			V_sum=0;
      I_sum=0;

      j = 0;
      k = 0;
			ref_count = 0;
			out_count = 0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
			//过零点检�???? 
				for (int i=0;i<128;i++)
				{
					if(ref_buffer[i] < trigger && ref_buffer[i+1]>trigger)
					{
						ref_trigger[j++] = i;
						ref_count++;
					}
					if(out_buffer[i] > trigger && out_buffer[i+1]<trigger)
					{
						out_trigger[k++] = i;
						out_count++;
					}
				}
				
				// 计算频率 
				
				uint16_t ref_trigger_sum = 0;
				ref_trigger_sum = ref_trigger[j-1] - ref_trigger[0];
				ref_freq = 6000*(ref_count-1)/(float)ref_trigger_sum;

				uint16_t out_trigger_sum = 0;
				out_trigger_sum = out_trigger[k-1] - out_trigger[0];
        out_freq = 6000*(out_count-1)/(float)out_trigger_sum;//TIM2的频率为6000Hz
    }
    

    // 载波定时器中断处
    if(tim1_flag == 1)
    {
			tim1_flag = 0;
			oled_count++;
			/*屏幕显示*/
			if(oled_count>=1000){
        oled_count=0;
        snprintf(ref_f_act, sizeof(ref_f_act), "%.2fHZ",ref_freq);
	      OLED_ShowString(60, 0, (uint8_t*)ref_f_act, sizeof(ref_f_act));

        snprintf(out_f_act, sizeof(out_f_act), "%.2fHZ",out_freq);
	      OLED_ShowString(60, 2, (uint8_t*)out_f_act, sizeof(out_f_act));

        snprintf(aar_act, sizeof(aar_act), "%d",autoreload);
	      OLED_ShowString(80, 4, (uint8_t*)aar_act, sizeof(aar_act));
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* 获取ADC转换结果 */
  if(hadc->Instance == ADC1)
  {
		dma_flag = 1;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 定时�????????3中断处理 */
  if(htim->Instance == TIM1)
  {
    tim1_flag = 1;
		// 根据频率更改 tim1
      if(out_freq > ref_freq&&autoreload<190)
      {
        __HAL_TIM_SET_AUTORELOAD(&htim1,autoreload++);
        __HAL_TIM_SET_COUNTER(&htim1,0);
      }
      else if(out_freq < ref_freq&&autoreload>120)
      {
        __HAL_TIM_SET_AUTORELOAD(&htim1, autoreload--);
        __HAL_TIM_SET_COUNTER(&htim1,0);
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, sin_k*sin_table[sin_index]+500);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, sin_k*sin_table[sin_index]+500);
      sin_index++;
      if(sin_index == 128)
      {
        sin_index = 0;
      }
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
