/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * Nikolay Belov, 2019
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "ir_codes.h"
#include "flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union {
	uint8_t adcod[2];	// adcod[0] - address, adcod[1] - code
	uint16_t adrcode;	// both
} ir_codes;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PAGE_63_ADDR 	0x0800FC00
#define CODES_SIGNATURE	0x66AA
#define CODES_MAX		25
#define PULSES_MAX		99
// duration connected state of Potentiometer, mS
#define KEY_PULSE		200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flag_pulse = 0;

uint8_t flagCounting = 0;
uint16_t ucTim2Flag = 0;

// IR packet variables
uint8_t irdata[33];             //Used to record the time between two falling edges
uint8_t receiveComplete;        //Receive Complete Flag Bits
uint8_t idx;                    //Number received for indexing
uint8_t startflag;              //Indicates start of reception
char sndbuf[128];
ir_codes butt_codes[CODES_MAX];	// signature + IR codes array

uint8_t teachingFlag = 0;		//
uint8_t teach_counter = 0;
//uint8_t ir_address = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
ir_codes decode_package(void);
uint8_t get_pulses(ir_codes adcom);
void led_blink(uint8_t cnt);
void BufferTransfer(char *buf, uint8_t sz);
void save_codes_to_flash(void);

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
	int8_t curr_count = 0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	receiveComplete = 0;
	startflag = 0;
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);	// break Potentiometer out

	// FLASH read
	uint32_t addr = PAGE_63_ADDR;
	for (uint8_t i = 0; i < (CODES_MAX); i++)
	{
		butt_codes[i].adrcode = flash_read(addr);
		addr += 2;
	}

	if (butt_codes[0].adrcode != CODES_SIGNATURE) {
		// saved array not exists -> Set Teaching mode
		teachingFlag = 1;
		teach_counter = 1;
		led_blink(4);
	}

  /**************************/
    /* Start pulse generation */
    /**************************/
    /* Enable channel 1 */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    //LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);

    /* Enable TIM1 outputs */
    LL_TIM_EnableAllOutputs(TIM1);

    /* Enable auto-reload register preload */
    LL_TIM_EnableARRPreload(TIM1);

    LL_TIM_EnableIT_UPDATE(TIM1);

    // Set Potentiometer to 0
    LL_TIM_DisableCounter(TIM1);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);	// count down
	LL_TIM_SetRepetitionCounter(TIM1, 100);
	LL_TIM_GenerateEvent_UPDATE(TIM1);	// update RCR
	flag_pulse = 1;
	LL_TIM_EnableCounter(TIM1);
	while (flag_pulse)
	{
		LL_mDelay(4);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	ir_codes ircode;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (receiveComplete)	// packet received
	  {
		  ircode = decode_package();	// 0xFF - broken IR package (code != ~code)
		  if (ircode.adcod[1] < 0xFF)
		  {
			  if (teachingFlag)
			  {
				  if (teach_counter < CODES_MAX)
				  {
					  butt_codes[teach_counter].adrcode = ircode.adrcode;
					  teach_counter++;
				  }
				  if (teach_counter == CODES_MAX)
				  {
					  led_blink(5);	// 5 - save codes and switch to work mode
					  save_codes_to_flash();
					  teachingFlag = 0;
					  teach_counter = 1;
				  }
				  else
				  {
					  led_blink(2);	// 2 - expect next code
				  }
			  }
			  else	// work mode
			  {
				  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);

				  int8_t npulses = get_pulses(ircode);
				  while (flag_pulse)
				  {
					  LL_mDelay(4);
				  }
				  uint8_t pulses = 0;
				  int8_t d = curr_count - npulses;
				  if ( d != 0)
				  {
					  if (d > 0)	// count down
					  {
						  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
						  pulses = d;
						  if (pulses > curr_count)
						  {
							  curr_count = 0;
						  }
						  else
						  {
							  curr_count -= pulses;
						  }
					  }
					  else	// count up
					  {
						  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
						  pulses = -d;
						  if ((curr_count + pulses) > PULSES_MAX)
						  {
							  curr_count = PULSES_MAX;
						  }
						  else
						  {
							  curr_count += pulses;
						  }
					  }
					  LL_TIM_SetRepetitionCounter(TIM1, pulses - 1);
					  LL_TIM_GenerateEvent_UPDATE(TIM1);	// update RCR
					  flag_pulse = 1;
					  LL_TIM_EnableCounter(TIM1);

					  receiveComplete = 0;
					  startflag = 0;

					  while (flag_pulse)
					  {
						  LL_mDelay(4);
					  }
				  }
				  // connect Potentiometer out
				  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
				  LL_mDelay(KEY_PULSE);
				  // break Potentiometer out
				  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);

				  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
			  }
		  }
		  receiveComplete = 0;
		  startflag = 0;
		  BufferTransfer(sndbuf, 32);
	  }	// if (receiveComplete)
	  else
	  {
		  // Check if Teaching mode button is pressed
		  if (!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4))
		  {
			  if (!teachingFlag)
			  {
				  // Set Teaching mode
				  teachingFlag = 1;
				  teach_counter = 1;
				  led_blink(4);
			  }
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
    Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(16000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(16000000);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 15;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 7;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 4;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetOnePulseMode(TIM1, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration  
  PA8   ------> TIM1_CH1 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 15;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 99;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */
  LL_TIM_EnableIT_UPDATE(TIM2);
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_GenerateEvent_UPDATE(TIM2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0|LL_GPIO_PIN_1);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE3);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void led_blink(uint8_t cnt)
{
	for (uint8_t i = 0; i < cnt; i++)
	{
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
		LL_mDelay(200);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
		LL_mDelay(200);
	}
}

void BufferTransfer(char *buf, uint8_t sz)
{
	for (uint8_t i = 0; i < sz; i++)
	{
		while (!LL_USART_IsActiveFlag_TXE(USART1)) {};
		if (i == (sz - 1))
		{
			LL_USART_ClearFlag_TC(USART1);
		}
		LL_USART_TransmitData8(USART1, buf[i]);
	}

	/* Wait for TC flag to be raised for last char */
	while (!LL_USART_IsActiveFlag_TC(USART1)) {};
}

void save_codes_to_flash(void)
{
	butt_codes[0].adrcode = CODES_SIGNATURE;
	uint32_t addr = PAGE_63_ADDR;
	flash_unlock();
	flash_erase_page(PAGE_63_ADDR);
	flash_lock();

	flash_unlock();
	for (uint8_t i = 0; i < CODES_MAX; i++)
	{
		flash_write(addr, butt_codes[i].adrcode);
		addr += 2;
	}
	flash_lock();
}

uint8_t get_pulses(ir_codes adcom)
{
	uint8_t res = 0;

	for (uint8_t i = 1; i < CODES_MAX; i++)
	{
		if ((butt_codes[i].adcod[1] == adcom.adcod[1])
				&& (butt_codes[i].adcod[0] == adcom.adcod[0]))
		{
			res = i * 4;
			break;
		}
	}
	return res;
}

ir_codes decode_package(void)
{
	uint8_t i, j;
    // idx starts with 1 to indicate that
	// the synchronization header time is not handled
    uint8_t idx = 1;
    uint8_t temp = 0;
    uint8_t remote_code[4];
    ir_codes res;

    for(i=0; i<4; i++)
    {
        for(j=0; j<8; j++)
        {
            if ((irdata[idx] >=8) && (irdata[idx] < 15))   //Represents 0
            {
            	temp = 0;
            }
            else if ((irdata[idx] >=18) && (irdata[idx]<25)) //Represents 1
            {
            	temp = 0x80; //temp = 1;
            }
            //remote_code[i] <<= 1;
            remote_code[i] >>= 1;
            remote_code[i] |= temp;
            idx++;
        }
    }
    // The array records control codes, each key is different
    sprintf(sndbuf, "A1:%02x A2:%02x C1:%02x C2:%02x Cmd:%03d\n",
    	remote_code[0], remote_code[1], remote_code[2], remote_code[3],
		remote_code[2]);

    if ((remote_code[2] | remote_code[3]) == 0xFF)
    {
    	res.adcod[0] = remote_code[0];
    	res.adcod[1] = remote_code[2];
    }
    else
    {
    	res.adrcode = 0xFFFF;
    }
    return res;
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
