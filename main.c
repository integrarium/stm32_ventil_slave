//I2C-датчик XGZP6897 плата сентябрь 2022

#include <math.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_flash.h>
#include <stm32f10x_bkp.h>
//#include <stm32f10x_rtc.h>

#include <I2C_slave.h>

#include <delay.h>
#include <main.h>


void init_gpio(void)
{
  GPIO_InitTypeDef gpio;
  GPIO_StructInit(&gpio);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &gpio);
}

// -----------------  DMA config ---------------------------------------

void DMA_Configuration(DMA_InitTypeDef *DMA_InitStructure)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); /* Enable DMA clock */

  //ADC1
  DMA_Cmd(DMA1_Channel1, DISABLE);
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure->DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
  DMA_InitStructure->DMA_MemoryBaseAddr = (uint32_t) ADC_DMA_buffer;
  DMA_InitStructure->DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure->DMA_BufferSize = 2;
  DMA_InitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure->DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure->DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure->DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);

}
//---------- adc конфиг  -----------------------------------------

void ADC_Config(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;

  /* Configure the ADC clock (must not exceed 14MHz) */
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* ADC Channel configuration */
  /* ADC GPIO clock enable */
//  RCC_APB2PeriphClockCmd(ADC_GPIO_CLK, ENABLE);  GPIOB

  /* Configure ADC1 Channel9 as analog input */
//  GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN ;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//  GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);

  /* ADC1 configuration */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  ADC_Init(ADC1, &ADC_InitStructure);

  // положение резистора
  /* ADC1 regular channel9 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); //самое медленное преобразование

  // температура МК
  /* ADC1 regular channel16 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_239Cycles5); //самое медленное преобразование

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */
  ADC_ResetCalibration(ADC1);

  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1))
  {
  }

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);

  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  ADC_DMACmd(ADC1, ENABLE); // DMA enabling -----------------------------------------------

  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); //Разрешение прерывания

  NVIC_EnableIRQ(ADC1_2_IRQn); //Разрешение прерывания

}
//-------------- прерывание АЦП ----------------------
//срабатывает 1 раз на 2 преобразования (по каждому каналу)
//данные достаёт DMA и складывает в буфер ADC_DMA_buffer
//а тут мы только накапливаем их в float-переменных
float temperature_data[2];

void ADC1_2_IRQHandler(void)
  {
  u8 i;
    for (i=0;i<2;i++)
    {

	          if (abs(temperature_data[i]-ADC_DMA_buffer[i])<50)
                  temperature_data[i] = temperature_data[i]*0.95+ADC_DMA_buffer[i]*0.05;
      else        temperature_data[i] = ADC_DMA_buffer[i];
    }
  }
// -----------------  RCC config ---------------------------------------

void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div4);
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* PLLCLK = 16MHz * 3 = 48 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_3);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08) {};
  }
}
//---------- crc16 для модбаса	-----------------------------------------
unsigned int calculateCRC(u8 *frame, unsigned char bufferSize)
{
  u16 temp, temp2, flag;
  u8 i, j;
  temp = 0xFFFF;
  for (i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
	temp ^= 0xA001;
    }
  }
  // Reverse byte order.
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}

//--------------------------------------------------
u16 mSecondCounter; //счётчик миллисекунд
u16 SecondCounter; //счётчик секунд

void SysTick_Handler(void)
{

	mSecondCounter++;
	if (mSecondCounter>=1000)  //=48e6/(2^16)
	  {
		mSecondCounter=0;
		SecondCounter++;

		if (VentConf->UV_on!=0) VentConf->UVSecondCounter++;
		if (VentConf->UVSecondCounter>=3600)
		  {
			VentConf->UVSecondCounter=0;
			VentConf->UVHours++;
		  }
	  }

	TimingDelay_Decrement();
}

/*******************************************************************************
* Function Name  : usart GPIO_Configuration
* Description	 : Configures the different GPIO ports.
* Input 	 : None
* Output	 : None
* Return	 : None
*******************************************************************************/
void GPIO_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure PB.02 pin 20 реле
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_2);

    // Configure PB.01 pin 20 реле
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);

    // Configure PB.15 pin28 выход sck на hx711
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);

    /* Configure USART2 TX_EN (PA.01) as out push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		  //rs-485     - разрешение передачи
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);

    /* Configure USART1 TX_EN (PA.08) as out push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		  //rs-485     - разрешение передачи
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);

    /* Configure USART2 Tx (PA.02) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		 //rs-485     - push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART2 Rx (PA.03) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		 //rs-485     - push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Configure USART2 PV (PA.05) as out push-pull */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//rs-485     - PV (power valid)
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOA, GPIO_Pin_5);			      //PV = 1

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	 //  Enable AFIO clock
	  /* Configure USART2 PV (PC.08) as alternate function push-pull */
//	    GPIO_PinRemapConfig( GPIO_FullRemap_TIM3 , ENABLE );

	  /* Configure LED (PC.15) as out push-pull */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOC, GPIO_Pin_15);

	  // Configure вход авария мотора (PC.13) as in     0 - нет аварии, 1 - авария
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure PWM-TIM3_CH3 (PB.0) as out push-pull */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Configure PWM-TIM3_CH4 (PB.1) as out push-pull */
//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Configure TIM3_ch1 (PA.6) as input pull up */
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    /* Configure TIM3_ch2 (PA.7) as input pull up */
		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		    GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* Configure TIM1_ch1 (PB.13) as out push-pull */
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);

}
/*******************************************************************************
* Function Name  : USART_Configuration
* Description	 : Configures the USART1.
* Input 	 : None
* Output	 : None
* Return	 : None
*******************************************************************************/
void USART_Configuration(int speed)
{
  USART_InitTypeDef USART_InitStructure;

/* USART1 configuration ------------------------------------------------------*/
  /* USART1 configured as follow:
	- BaudRate = 115200 baud
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	- USART Clock disabled
	- USART CPOL: Clock is active low
	- USART CPHA: Data is captured on the middle
	- USART LastBit: The clock pulse of the last data bit is not output to
			 the SCLK pin
  */
  USART_InitStructure.USART_BaudRate = speed;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;  // нет четности, для бита четности нужно длину структуры делать 9бит
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);

  /* Enable USART1 */
  USART_Cmd(USART2, ENABLE);
}

/*******************************************************************************
* Function Name  : UARTSend
* Description	 : Send a string to the UART.
* Input 	 : - pucBuffer: buffers to be printed.
*		 : - ulCount  : buffer's length
* Output	 : None
* Return	 : None
*******************************************************************************/
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
    USART_SendData(USART2, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
//	USART_SendData(USART1, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
	/* Loop until the end of transmission */
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
//	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}
    }
}


/*******************************************************************************
* Function Name  : usart_rxtx
* Description	 : Print "Welcome to CooCox!"  on Hyperterminal via USART1.
* Input 	 : None
* Output	 : None
* Return	 : None
*******************************************************************************/
void usart_init(void)
{
//	    const unsigned char menu[] = " Welcome to CooCox!\r\n";

	    /* Enable USART1, USART2 and GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	    /* NVIC Configuration */
//	    NVIC_Configuration();


	    /* Configure the GPIOs */
	    GPIO_Configuration();

	    /* Configure the USART2 */
	    USART_Configuration(ModBusUartSpeed); //из main.h

	    /* Enable the USART2 Receive interrupt: this interrupt is generated when the
		 USART2 receive data register is not empty */
	    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//Прерывание по приему
	    USART_ITConfig(USART2, USART_IT_TC, ENABLE);	//Прерывание по завершению отправки

	    NVIC_EnableIRQ(USART2_IRQn); //Включаем прерывания от UART
	    NVIC_SetPriority(USART2_IRQn, 0); //Прерывание от UART, приоритет 0, самый высокий
//	    USART2->CR1 |= USART_CR1_RXNEIE;
//	    USART2->CR1 |= USART_CR1_; //Прерывание по завершению отправки
	    USART_ClearFlag (USART2, USART_FLAG_TC);


	    /* print welcome information */
//	    UARTSend(menu, sizeof(menu));
}

/**
  * @brief  Initializes the SPI Interface used to drive the LCD and uSD card
  *	    available on adafruit 1.8" TFT shield.
  * @note   This function must by called by the application code before to initialize
  *	    the LCD and uSD card.
  * @param  None
  * @retval None
  */

//----------- RTC -------------------------------
unsigned char RTC_Init(void)
{
	// Включить тактирование модулей управления питанием и управлением резервной областью
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	// Разрешить доступ к области резервных данных
	PWR_BackupAccessCmd(ENABLE);
	// Если RTC выключен - инициализировать
/*
	if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)
	{
		// Сброс данных в резервной области
//		  RCC_BackupResetCmd(ENABLE);
//		  RCC_BackupResetCmd(DISABLE);
		// Установить источник тактирования кварц 32768
//		  RCC_LSEConfig(RCC_LSE_ON);
//		  while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);
		RTC_SetPrescaler(125000-1); // Устанавливаем делитель, чтобы часы считали секунды
		// Включаем RTC
		RCC_RTCCLKCmd(ENABLE);
		// Ждем синхронизацию
		RTC_WaitForSynchro();
		return 1;
	}
*/
	return 0;
}
//----------------------------------------------------------------------------------
void Timer2_Config(int MCU_Frequency)
  {

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	  TIM_TimeBaseInitTypeDef timer_base;
	  TIM_TimeBaseStructInit(&timer_base);
	  timer_base.TIM_Prescaler = 115200/ModBusUartSpeed; //(MCU_Frequency/1000000) - 1;
	  TIM_TimeBaseInit (TIM2, &timer_base);
	  TIM_ITConfig (TIM2, TIM_IT_Update, ENABLE);

	  NVIC_EnableIRQ(TIM2_IRQn); //Разрешение прерывания

  }
//----------------------------------------------------------------------------------
void Timer1_Config(int MCU_Frequency)
  {

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	  TIM_TimeBaseInitTypeDef timer_base;
	  TIM_TimeBaseStructInit(&timer_base);
	  timer_base.TIM_Prescaler = 0xffff;
	  timer_base.TIM_Period = 9;

	  TIM_TimeBaseInit (TIM1, &timer_base);

	  TIM_OCInitTypeDef outputChannelInit;

	  TIM_OCStructInit(&outputChannelInit);
	  outputChannelInit.TIM_Pulse = 4;			      //
	  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	  outputChannelInit.TIM_OutputState = TIM_OutputState_Disable;
	  outputChannelInit.TIM_OutputNState = TIM_OutputNState_Enable;
	  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	  TIM_OC1Init(TIM1, &outputChannelInit);
	  TIM1->BDTR|=0x8000;


	  TIM_Cmd(TIM1, ENABLE);

  }
//----------------------------------------------------------------------------------
void Timer3_Config(int MCU_Frequency)
  {

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	  TIM_TimeBaseInitTypeDef timer_base;
	  TIM_TimeBaseStructInit(&timer_base);
	  timer_base.TIM_Prescaler = 0;
	  TIM_TimeBaseInit (TIM3, &timer_base);

	  TIM_OCInitTypeDef outputChannelInit;

	  TIM_OCStructInit(&outputChannelInit);
	  outputChannelInit.TIM_Pulse = 2;			      //
	  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	  outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	  outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
	  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	  TIM_OC3Init(TIM3, &outputChannelInit);

	  outputChannelInit.TIM_Pulse = 2;			      //
	  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	  outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	  outputChannelInit.TIM_OutputNState = TIM_OutputNState_Disable;
	  outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	  TIM_OC4Init(TIM3, &outputChannelInit);

	  TIM_ICInitTypeDef inputChannelInit;

	  TIM_ICStructInit(&inputChannelInit);
	  inputChannelInit.TIM_Channel = TIM_Channel_1; 			   //
	  inputChannelInit.TIM_ICPolarity = TIM_ICPolarity_Rising;
	  inputChannelInit.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  inputChannelInit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  inputChannelInit.TIM_ICFilter = 0;
	  TIM_ICInit(TIM3, &inputChannelInit);

	  inputChannelInit.TIM_Channel = TIM_Channel_2; 			   //
	  inputChannelInit.TIM_ICPolarity = TIM_ICPolarity_Rising;
	  inputChannelInit.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  inputChannelInit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  inputChannelInit.TIM_ICFilter = 0;
	  TIM_ICInit(TIM3, &inputChannelInit);

//	    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	  TIM_ITConfig (TIM3, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

	  NVIC_EnableIRQ(TIM3_IRQn); //Разрешение прерывания
	  TIM_Cmd(TIM3, ENABLE);
  }
//-------------- прерывание уарта ----------------------
u8 NumBytesToSend;

void USART2_IRQHandler(void)
{

	  u8 temp;
	    if ((USART2->SR) & USART_FLAG_RXNE)  //пришёл байт?
		  {
	      temp=USART2->DR;		 //принимаем байт
	      if ((RTCounter<100) & !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1))  //если буфер не кончился и передача не включена
		{
		    frame[RTCounter++]=temp;	     //кладём байт в буфер
	    TIM_SetCounter(TIM2,0xf000);     //сброс счётчика времени
	    TIM_Cmd(TIM2, ENABLE);	   //вкл таймера
		}
		  }
	    if ((USART2->SR) & USART_FLAG_TC)  //ушёл байт?
	      {
		if (RTCounter<NumBytesToSend)
		{
		    USART_SendData(USART2, frame[RTCounter++]); 	 //отправка следующего байта
		}
		else
		  {
				//завершение передачи
		    USART_ClearFlag (USART2, USART_FLAG_TC);  //чистим флаг
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);	 // передача выкл
			RTCounter=0;
		  }

	      }
}

//------------- прерывание таймеров -------------------

s32 integrator;
u32 temp_tim3_1,temp_tim3_2;
u16 HighWordTimCnt;

void TIM3_IRQHandler (void)
  {
    s32 err_pres, power;
    u16 tim_flags;
    u32 temp_tim3;

    tim_flags = TIM3->SR;
//    		TIM_GetFlagStatus(TIM3, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2);
    TIM_ClearFlag (TIM3, TIM_FLAG_Update | TIM_FLAG_CC1 | TIM_FLAG_CC2);	    //сброс флага
//    if (tim_flags != TIM_IT_Update)
//      {
//    	 CorrConf[0]->FanTimCapt++;

//      }
    temp_tim3 = ((HighWordTimCnt+1)<<16);
    if (((temp_tim3-temp_tim3_1)>>10) > 0xffff) CorrConf[0]->FanSpeed = 0; //скорость меньше порога
    if (((temp_tim3-temp_tim3_2)>>10) > 0xffff) CorrConf[1]->FanSpeed = 0; //скорость меньше порога

    if (tim_flags & TIM_FLAG_Update)  //частота = 48 МГц/65536 = 732 Гц
    {

	err_pres=0;
	HighWordTimCnt++;

	if ((CorrConf[0]->ustav>0) && (VentConf->fan_on>0) ) //если уставка не ноль и включение разрешено - работает регулятор
      {
      err_pres=VentConf->cur_flaw;   //регулируем по потоку
	  err_pres-=(CorrConf[0]->ustav);   //разница потока
	  integrator-=err_pres; 	  //накопление ошибки в интеграторе
	 // integrator+=16;
	  if (integrator<-(0xffff<<8)) integrator=-(0xffff<<8);    //ограничение интегратора
      if (integrator> (0xffff<<8)) integrator= (0xffff<<8);    //ограничение интегратора
//	  power=(integrator>>5)-(err_pres<<6);	//считаем мощность. коэффициенты заданы степенями двойки
	  power=(integrator>>8);  //считаем мощность. коэффициенты заданы степенями двойки
	  if (power<0) power=0; 		//мощность не может быть отрицательной
      if (power>0xffff) power=0xffff;	    //мощность не может быть > 100%
      }
    else
      {
	power=0;
	integrator=0;
      }
    if ((CorrConf[0]->WorkMode & 1)==0) //режим "регулятор расхода"
      {
        CorrConf[0]->power=power & 0xffff;
        VentConf->fan_power=power*10000/0xffff;
        CorrConf[0]->error=err_pres & 0xffff;
    	TIM3->CCR3=power;  //перезапись расчитанной мощности в таймер
        TIM3->CCR4-=1;
      }
    }
    else if (tim_flags & TIM_FLAG_CC1)
      {
      temp_tim3 = (HighWordTimCnt<<16) | TIM3->CCR1;
//      CorrConf[0]->FanTimCapt = (temp_tim3-temp_tim3_1)>>10;
      if (((temp_tim3-temp_tim3_1)>>10) < 0xffff) CorrConf[1]->FanSpeed = 48000000/(temp_tim3-temp_tim3_1) ; //считаем скорость
      temp_tim3_1=temp_tim3;
      }
    else if (tim_flags & TIM_FLAG_CC2)
      {
      temp_tim3 = (HighWordTimCnt<<16) | TIM3->CCR2;
//      CorrConf[1]->FanTimCapt = (temp_tim3-temp_tim3_2)>>10;
      if (((temp_tim3-temp_tim3_2)>>10) < 0xffff) CorrConf[0]->FanSpeed = 48000000/(temp_tim3-temp_tim3_2) ; //скорость
      temp_tim3_2=temp_tim3;
      }

  }


void TIM2_IRQHandler (void)
{
u16 crc16, addr;
u16 temp, i;
u16 *p_devconf;

	//пакет принят

	TIM_Cmd(TIM2, DISABLE); 	//выкл таймера
	TIM_ClearFlag (TIM2, TIM_FLAG_Update);	    //сброс флага
    if (RTCounter>7)  //пакеты короче 8ми байт игнорируем
      {
      crc16 = frame[RTCounter-2]<<8|frame[RTCounter-1];
      if (crc16==calculateCRC(frame, RTCounter-2)) //crc совпал?
	{
	if ( (frame[0]==((CorrConf[0]->OurMBAddress)&0xff)) || (frame[0]==0)/*| (frame[0]==((CorrConf[1]->OurMBAddress)&0xff))*/)  //проверка адреса
	  {
//		if (frame[0]==((CorrConf[0]->OurMBAddress)&0xff))
//		  {
			p_devconf=(u16 *)(& (dev_conf[0]));
//		  }
//		else p_devconf=(u16 *)(& (dev_conf[1])); //обращение по второму модбас-адресу

		switch (frame[1])
		{
		case 3: 	//запрос на выдачу массива
		if (frame[0]==((CorrConf[0]->OurMBAddress)&0xff)) //запросы на чтение - только по основному адресу
		  {
		  temp=(frame[4]<<8)|frame[5];
		  addr=(frame[2]<<8)|frame[3];
			if (temp>100) temp=100;	//если запрошено больше 100 регистров, то отдаём только первые 100 (буфер-то не резиновый)
		  frame[2]=temp*2;		//кол-во байт данных в пакете
		  for (i=0; i<temp; i++)
		    {
			  frame[i*2+4]=(p_devconf[i+addr])&0xff;		   //копируем данные в отправочный массив
			  frame[i*2+3]=(p_devconf[i+addr])>>8;
		    }
		  NumBytesToSend=temp*2+5;  //длина ответа c crс
		  }
		break;
		case 6: 	//запрос на запись регистра
		addr=(frame[2]<<8)|frame[3];	      //считаем адрес
		(p_devconf[addr])=(frame[4]<<8)|frame[5];	//копируем дату
			NumBytesToSend=8;  //длина ответа c crc (ответ равен запросу)
			break;
		default:
		frame[1]|=0x80;  //устанавливаем бит ошибки
		frame[2]=1;	 //неподдерживаемая команда
		NumBytesToSend=5;  //длина ответа c crc
		break;
		}

	  crc16=calculateCRC(frame, NumBytesToSend-2);
	  frame[NumBytesToSend-2]=crc16>>8;
	  frame[NumBytesToSend-1]=crc16 & 0xff;
	  GPIO_SetBits(GPIOA, GPIO_Pin_1);   // передача вкл
	  USART_SendData(USART2, frame[0]);	     //первый байт пошёл, остальные - в прерывании от уарта
	  RTCounter=1;	  //число отправленных байт
	  }
	}

      }
    if (!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1))  //если передача не включена
		RTCounter=0;				 //готовимся к приёму следующего пакета
}


//	  GPIO_SetBits(GPIOA, GPIO_Pin_1);   // передача вкл
//	USART_SendData(USART2, 0x55);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
//	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)  //ждём конца передачи
//		{
//		}
//	  GPIO_ResetBits(GPIOA, GPIO_Pin_1);	   // передача выкл

//--------- i2c -------------------------------------------------------------------------

u8 i2c_error;

void init_I2C(I2C_TypeDef* I2Cx)
  {
    I2C_InitTypeDef i2c;
    // настройка I2C
    i2c.I2C_ClockSpeed = 100000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x15;  //для мастера любой, кроме 0
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
  }

// i2c инициализация
//****** по I2C2 идёт обмен с главным МК, тут только слейв *************************************************************
/*
void init_I2C2(void)
{
	GPIO_InitTypeDef gpio;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    init_I2C(I2C2);
	// ноги I2C2
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD ;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

	I2C_Cmd(I2C2, ENABLE);
}
*/
//********** по I2C1 работают датчики давления. порт мультиплексируется между PB6-PB7 и PB8-PB9 *****************************
void init_I2C1(void)
{
	GPIO_InitTypeDef gpio;
	// Включаем тактирование нужных модулей
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


    init_I2C(I2C1);

    // ноги I2C1
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU ;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    // ноги I2C1 2й вариант, включим его пока на вход с подтяжкой вверх
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD ;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_PinRemapConfig( GPIO_Remap_I2C1 , ENABLE );

//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C1);
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C1);
    // включаем I2C1
    I2C_Cmd(I2C1, ENABLE);
}
//*******************************************************************************************************
void I2C_StartTransmission(I2C_TypeDef* I2Cx, uint8_t transmissionDirection,  uint8_t slaveAddress)
{
    // На всякий слуыай ждем, пока шина осовободится
	SetTimingDelay(10);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))	  if (GetTimingDelay()==0) { i2c_error=1;break;  }
    if	(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
//	while(1); //пипец

	}

    // Генерируем старт
    I2C_GenerateSTART(I2Cx, ENABLE);
    // Ждем пока взлетит нужный флаг
	SetTimingDelay(10);
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		  if (GetTimingDelay()==0) {i2c_error=2;break;}
    // Посылаем адрес подчиненному
    I2C_Send7bitAddress(I2Cx, slaveAddress, transmissionDirection);
    // А теперь у нас два варианта развития событий - в зависимости от выбранного направления обмена данными
	SetTimingDelay(10);
    if(transmissionDirection== I2C_Direction_Transmitter)
    {
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
				if (GetTimingDelay()==0) {i2c_error=3;break;}
    }
    if(transmissionDirection== I2C_Direction_Receiver)
    {
      while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			if (GetTimingDelay()==0) {i2c_error=4;break;}

    }
}

/***************************************************************************************/
void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data)
{
    I2C_SendData(I2Cx, data);
	SetTimingDelay(10);
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	     if (GetTimingDelay()==0) {i2c_error=5;break;}
}
/***************************************************************************************/
uint8_t I2C_ReadData(I2C_TypeDef* I2Cx)
{
    uint8_t data;

//ждём
	SetTimingDelay(10);
     while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
	   if (GetTimingDelay()==0) {i2c_error=6;break;}

//опять ждём
	SetTimingDelay(10);
	while(!(I2Cx->SR1 & I2C_SR1_RXNE)) if (GetTimingDelay()==0) {i2c_error=7;break;}

//читаем
	data = I2Cx->DR;

	return data;
}

/***************************************************************************************/
void XGZP_WriteByte(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t data)
  {
    if (i2c_error>0) return;
	I2C_StartTransmission(I2Cx,I2C_Direction_Transmitter,0xDA);
    I2C_WriteData(I2Cx,addr);
    I2C_WriteData(I2Cx,data);
    I2C_GenerateSTOP(I2Cx, ENABLE);
  }
/***************************************************************************************/
uint8_t XGZP_ReadByte(I2C_TypeDef* I2Cx, uint8_t addr)
  {
    if (i2c_error>0) return 0;
    uint8_t tmp;
	I2C_StartTransmission(I2Cx,I2C_Direction_Transmitter,0xDA);
    I2C_WriteData(I2Cx,addr);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    I2C_StartTransmission(I2Cx,I2C_Direction_Receiver,0xDA);
    I2C_AcknowledgeConfig(I2Cx,DISABLE);
    tmp=I2C_ReadData(I2Cx);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    return tmp;
  }
/***************************************************************************************/
void hx711_clkpulse ()
  {
	 GPIOB->ODR|=(GPIO_ODR_ODR15);    //SCK = 1    импульс начало
	 GPIOB->ODR|=(GPIO_ODR_ODR15);    //SCK = 1    импульс начало
	 GPIOB->ODR|=(GPIO_ODR_ODR15);    //SCK = 1    импульс начало
	//Count=Count^0x800000;
	 GPIOB->ODR&=~(GPIO_ODR_ODR15);    //SCK = 0  импульс конец
	 GPIOB->ODR&=~(GPIO_ODR_ODR15);    //SCK = 0  импульс конец
	 GPIOB->ODR&=~(GPIO_ODR_ODR15);    //SCK = 0  импульс конец
  }

//------ FLASH --------------------------------------------------------

uint32_t Read_Flash(uint32_t Data_adr)
{
   return *(uint32_t*) Data_adr;

}

//поиск позиции последнего непустого слова в странице flash
u16 FlashSearch (uint8_t ParNum)
  {
  u16 i;
  u32 PageAddr=FlashLastPageAddr-ParNum*FlashPageSize;
  for (i=FlashPageSize/2-1; i>0; i--)
    {
    if (((uint16_t*)PageAddr)[i] < 0xffff)  break;
    }
  return i;
  }

// чтение параметра по его номеру из флеша
u16 ReadFlashParam(uint8_t ParNum)
  {
	u16 idx;
	idx = FlashSearch(ParNum);
	return ((uint16_t*)(FlashLastPageAddr-ParNum*FlashPageSize))[idx];

//    return *(uint16_t*)(PageAddr+FlashPageSize-2-i);
  }

// запись параметра с заданным номером во флеш
void WriteFlashParam(uint8_t ParNum, u16 ParValue)
  {
	u16 idx;
	idx = FlashSearch(ParNum);
    FLASH_Unlock();
	if ((idx==FlashPageSize/2-1) && (((uint16_t*)(FlashLastPageAddr-ParNum*FlashPageSize))[idx])<0xffff)
	  {
	  FLASH_ErasePage(FlashLastPageAddr-ParNum*FlashPageSize);	  //страница полностью записана - надо стереть
	  }
    if (((uint16_t*)(FlashLastPageAddr-ParNum*FlashPageSize))[idx]==0xffff)
		  FLASH_ProgramHalfWord  ( FlashLastPageAddr-ParNum*FlashPageSize,  ParValue  );   //чистая страница - пишем в начало
    else
	   FLASH_ProgramHalfWord  ( FlashLastPageAddr-ParNum*FlashPageSize+2*idx+2,  ParValue  );  //пишем в свободную ячейку (следующую за занятой)
	FLASH_Lock();
  }

void FlashFill (void)
	{
    u16 i, temp16;
	for (i=0; i<ParNumTableSize; i++)
	  {
	  temp16 = ReadFlashParam(i);
	  if (temp16==0xffff)  //во флеше пусто - запишем туда из озу
		WriteFlashParam(i, dev_conf[0].devConfig[ParNumTable[i]]);     //ram->flash
	  else dev_conf[0].devConfig[ParNumTable[i]]=ReadFlashParam(i); //иначе flash->ram
	  }
	}

void FlashSyncronize (void)
  {
    u16 i, temp16;
	for (i=0; i<ParNumTableSize; i++)
	  {
	  temp16 = ReadFlashParam(i);
	  if (temp16!=dev_conf[0].devConfig[ParNumTable[i]])  //если есть отличие
		  WriteFlashParam(i, dev_conf[0].devConfig[ParNumTable[i]]);	 //ram->flash
	  }
  }

//------------------------------------------------------------------------------------

void BKPWriteAll(void)
  {
    u16 i, temp16;
    u16 buf[10];
    for (i=0; i<ParNumTableBKPSize; i++) //запись всех параметров в BKP
      {
      buf[i] = dev_conf[0].devConfig[ParNumTableBKP[i]];
      BKP_WriteBackupRegister(BKP_DR1+i*4,buf[i]); //ram->bkp
      }
    temp16 = calculateCRC(buf, 18); //новая crc
    BKP_WriteBackupRegister(BKP_DR1+9*4,temp16); //запись CRC

  }

void BKPFill (void)
	{
    u16 i, temp16;
    u16 buf[10];
	for (i=0; i<BKPSize; i++) //читаем весь BKP
	  {
	  buf[i] = BKP_ReadBackupRegister(BKP_DR1+i*4);
	  }
    temp16 = calculateCRC(buf, (BKPSize-1)*2); //Проверяем целостность данных

    if (temp16!=buf[BKPSize-1])  //если данные BKP неверные - запишем туда из озу
      {
      BKPWriteAll();
      VentConf->con_alarm|=1;
      }
    else
      {
    	for (i=0; i<ParNumTableBKPSize; i++) dev_conf[0].devConfig[ParNumTableBKP[i]]=buf[i]; //читаем все параметры из BKP
        VentConf->con_alarm&=~1;

	  }
	}
/*
void BKPSyncronize (void)
  {
    u16 i, temp16;
	for (i=0; i<ParNumTableSize; i++)
	  {
	  temp16 = ReadFlashParam(i);
	  if (temp16!=dev_conf[0].devConfig[ParNumTable[i]])  //если есть отличие
		  WriteFlashParam(i, dev_conf[0].devConfig[ParNumTable[i]]);	 //ram->flash
	  }
  }
*/

//------------------------------------------------------------------------------------

u32 compil_date = __DATE__;

int main(void)
{
	u16 temp16;
	u16 i;
    s16 press;
    s32 press32;
    s16 temper;
    u32 Count;
    u16 Count_time;
    char debugstr[100];
    u16 PrevmSecondCounter; //предыдущее значение счётчика миллисекунд
    u8 hx710_phase=0;
    u16 GateWayState=0;

	//u16 kf_press1, kf_press2; // переменные коэф пропорциональности

    integrator=0;
    RCC_ClocksTypeDef RCC_Clocks;

	RCC_Configuration();
	init_gpio();
    __disable_irq(); //Глобальное выключение прерывания

    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

    RTC_Init();
    ADC_Config();
    DMA_Configuration(&DMA_InitStructure);

    Timer1_Config(RCC_Clocks.HCLK_Frequency);
    Timer2_Config(RCC_Clocks.HCLK_Frequency);
    Timer3_Config(RCC_Clocks.HCLK_Frequency);
    usart_init();

    init_I2C1();
    //    init_I2C2();
    CorrConf[0]=&dev_conf[0];
    CorrConf[1]=&dev_conf[1];
    VentConf=&dev_conf[2];
    // начальные значения настроек в случае отсутствия их во флеше
    CorrConf[0]->OurMBAddress = 2; //ardress modbus (датчик перепада на фильтре)
    //    CorrConf[1]->OurMBAddress=3; //больше не участвует в работе // ardress modbus (датчик потока на двигателе)
    //    CorrConf[0]->temper_base=20; //больше не участвует в работе // температура базовая
    //    CorrConf[1]->temper_base=20; //температура базовая больше нет. настройка по первому регистру
    CorrConf[0]->WorkMode=1;   //вкл режим "шлюз" односторонний, уф включен
    VentConf->Kfactor = 77; //
    VentConf->flaw_set1 = 1000; //уставка потока 1
    VentConf->flaw_set1 = 500;  //уставка потока 2
    VentConf->min_pres = 60; //нижнее давление для определения аварии фильтра
    VentConf->max_pres = 350; //верхнее давление для определения аварии фильтра
    VentConf->Version = 401; //Номер версии прошивки
    CorrConf[0]->FFMnumber=1; //количество ФВМ
    CorrConf[0]->InFanPower=1000; // мощность мотора
    VentConf->fan_on=0;
//      VentConf->UVSecondCounter=BKP_ReadBackupRegister(BKP_DR1); //наработку в секундах берём из бакап-озу (на батарейке)
    CorrConf[0]->PCA9534_3 =0xf0; //старший полубайт PCA9534 на выход
    CorrConf[0]->LightTime =20; //Время освещения
    CorrConf[0]->BlowTime =30;  // Время обработки
    CorrConf[0]->CheckTime =3;  // Время до проверки

    FlashFill();
    BKPFill();

    GA_Adress=CorrConf[0]->OurMBAddress>>1;  //адрес на слейв-и2ц = адресу на модбасе
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_DMA_buffer[0]=0;
    ADC_DMA_buffer[1]=0;

     Init_I2C_Slave();

    __enable_irq(); //Глобальное включение прерывания

//    SW_I2C_initial();
//    i2c_port_initial(SW_I2C2);


    // три старших бита порта PCA9534 на выход
//    SW_I2C_WriteControl_8Bit(SW_I2C2,0x40,3,0x1f);

//            I2C_StartTransmission(I2C2,I2C_Direction_Transmitter,0x40);
//            I2C_WriteData(I2C2,3);
//            I2C_WriteData(I2C2,0x1f);
//            I2C_GenerateSTOP(I2C2, ENABLE);
    SecondCounter=0;

    while (1)
    {
    	FlashSyncronize();
    	BKPWriteAll();

    	//реостат
    	CorrConf[0]->rheostat=temperature_data[0];
    		//(3469.2/(log(temperature_data[0]/(4096-temperature_data[0]))+3469.2/298.15)-273.15)*100;
    	//температура МК
    	CorrConf[0]->temperature= ((1750.0-temperature_data[1])/5+25)*100;
    	if (CorrConf[0]->OurMBAddress==0) CorrConf[0]->OurMBAddress=1;

    	if ((VentConf->filter_alarm>0) || (VentConf->fan_alarm>0)) VentConf->main_alarm=1;
    	else VentConf->main_alarm=0;

    	PrevmSecondCounter=mSecondCounter;

    	if (((CorrConf[0]->WorkMode & 1)==0)          //режим регулятора
    			&& (VentConf->fan_power==0))
    		VentConf->fan_alarm = 0; // сброс аварии мотора

    	if (VentConf->set_select == 0) CorrConf[0]->ustav = VentConf->flaw_set1;
    	else CorrConf[0]->ustav = VentConf->flaw_set2;

    	if (VentConf->LightOn==1) GPIO_SetBits(GPIOB, GPIO_Pin_2);
    	else  GPIO_ResetBits(GPIOB, GPIO_Pin_2);

    	if (VentConf->UV_on==1) GPIO_SetBits(GPIOB, GPIO_Pin_1);
    	else  GPIO_ResetBits(GPIOB, GPIO_Pin_1);

//	  if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)==0) VentConf->fan_alarm = 0;
//	  else VentConf->fan_alarm = 1;


/*	temp16=BKP_ReadBackupRegister(BKP_DR1);
	if (VentConf->UVSecondCounter!=temp16)
	  {
	  BKP_WriteBackupRegister(BKP_DR1,VentConf->UVSecondCounter);
	  }
*/
// --------------------- первый датчик -------------------------------------------------------------------------

    	i2c_error=0;
    	XGZP_WriteByte(I2C1, 0xa5, XGZP_ReadByte(I2C1, 0xa5)&0xfd); //Set ADC output calibrated Data
    	XGZP_WriteByte(I2C1, 0x30, 0x0A); //indicate a combined conversion (once temperature conversion immediately followed by once sensor signal conversion)

    	while ((XGZP_ReadByte(I2C1, 0x30) & 0x08) > 0); //Judge whether Data collection is over
	//     press=XGZP_ReadByte(I2C1, 0x5) << 16;
    	press=XGZP_ReadByte(I2C1, 0x6) << 8;
	    press|=XGZP_ReadByte(I2C1, 0x7) ;

//	   if (press&0x8000) press |=0xffff0000;  //расширение знака

	    temper=XGZP_ReadByte(I2C1, 0x9) << 8;
	    temper|=XGZP_ReadByte(I2C1, 0xa);
	    Delay(1);

	    if (i2c_error==0)
	    {
	    	CorrConf[0]->rawpress_i2c = press;
	    	CorrConf[0]->temper_i2c = temper*100/256; // Получаем градусы * 100
//	 CorrConf->press_i2c = ((s32)(temp16)*5000/32767-5000-CorrConf->press_offset);
//	  CorrConf[0]->press_i2c = (((temp16+1024)/60)-500-CorrConf[0]->press_offset)*(1+(CorrConf[0]->kf_press/100));
//		  CorrConf[0]->press_i2c = ( (temp16+1024)/60 -500-CorrConf[0]->press_offset)*(100+CorrConf[0]->kf_press)/100;

	      //CorrConf[0]->press_i2c =(press-CorrConf[0]->press_offset)/(1+CorrConf[0]->kf_press);// для уменьшения в К раз - не работает

	    	press32=press;
	    	press32-=CorrConf[0]->press_offset;
	    	if (press32>0x7fff) press32 =0x7fff;
	    	if (press32<-0x8000) press32 =-0x8000;
//	CorrConf[0]->tcomp_offset = (2.5*(CorrConf[0]->temper_base/100 - CorrConf[0]->temper_i2c/100)); // уход нуля
//	    CorrConf[0]->tcomp_diap = (press32 + CorrConf[0]->tcomp_offset ) * (100+CorrConf[0]->kf_press)/100 * (1.65 * (CorrConf[0]->temper_i2c - CorrConf[0]->temper_base)/CorrConf[0]->press_diap)/100; //уход диапазона
//	    CorrConf[0]->press_i2c = (press32 + CorrConf[0]->tcomp_offset ) * (100+CorrConf[0]->kf_press)/100 + CorrConf[0]->tcomp_diap; //сумма
	    	CorrConf[0]->filter = ((CorrConf[0]->filter*19)+   press32)/20; //с фильтрацией
//	  CorrConf[0]->filter = ((CorrConf[0]->press_i2c*19)+  ( press32 + CorrConf[0]->tcomp_offset ) * (100+CorrConf[0]->kf_press)/100 + CorrConf[0]->tcomp_diap)/20; //с фильтрацией
	//CorrConf[0]->press_i2c =   (( press32 + CorrConf[0]->tcomp_offset ) * (100+CorrConf[0]->kf_press)/100 + CorrConf[0]->tcomp_diap); //с фильтрацией
	    //CorrConf[0]->press_i2c = (press - CorrConf[0]->press_offset + CorrConf[0]->tcomp_offset ) * (100+CorrConf[0]->kf_press)/100; //сумма

	    	// записываем значение датчика давления первого канала в регистр
	    	CorrConf[0]->press_i2c=CorrConf[0]->filter*10/16;

	    	if (CorrConf[0]->press_i2c > 0) VentConf->cur_flaw=sqrt(abs(CorrConf[0]->press_i2c))*VentConf->Kfactor/3.16;
	    	else VentConf->cur_flaw=-sqrt(abs(CorrConf[0]->press_i2c))*VentConf->Kfactor/3.16;

	    	CorrConf[0]->i2c_success_count++;
	    	CorrConf[0]->i2c_sens_error = 0; //нет ошибок
	    }
	    else
	    {
	    	CorrConf[0]->i2c_sens_error = i2c_error; //ошибка датчика
	    	i2c_error=0;
	    	I2C1->CR1 |= I2C_CR1_SWRST; //reset
	    	// конфигурируем ножки на выход (иногда слетала конфигурация)
	    	GPIOB->CRH |= GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1 |GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1;
	    	if (!((GPIOB->IDR) & (1<<9))) //слейв держит шину
	    	{
	    		while  (!((GPIOB->IDR) & (1<<9))) //PB9
	    		{
	    			GPIOB->CRH &= ~ GPIO_CRH_CNF8_1; //шлём клоки на PB8, пока слейв не отпуситит шину
	    			Delay(1);
	    			GPIOB->CRH |= GPIO_CRH_CNF8_1;
	    			Delay(1);
	    		}
	    	}
	    	Delay(10);
	    	I2C1->CR1 &= ~I2C_CR1_SWRST; //отпускаем reset
	    	init_I2C(I2C1);	   //инит i2c после ресета
	    }

// -------------------------- второй датчик ------------------------------------------------------------------------------

	    //отключаемся от 1го датчика
	    GPIOB->ODR |= GPIO_ODR_ODR8 | GPIO_ODR_ODR9;
	    GPIOB->CRH &= ~GPIO_CRH_CNF8_0 & ~GPIO_CRH_MODE8_1 & ~GPIO_CRH_MODE8_0 & ~GPIO_CRH_CNF9_0 & ~GPIO_CRH_MODE9_1 & ~GPIO_CRH_MODE9_0;
	    //подключаемся к 2му датчику
	    GPIO_PinRemapConfig( GPIO_Remap_I2C1 , DISABLE );
	    GPIOB->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0;
	    GPIOB->ODR &= ~GPIO_ODR_ODR6 & ~GPIO_ODR_ODR7;
	    XGZP_WriteByte(I2C1, 0xa5, XGZP_ReadByte(I2C1, 0xa5)&0xfd); //Set ADC output calibrated Data
	    XGZP_WriteByte(I2C1, 0x30, 0x0A); //indicate a combined conversion (once temperature conversion immediately followed by once sensor signal conversion)

	    while ((XGZP_ReadByte(I2C1, 0x30) & 0x08) > 0); //Judge whether Data collection is over
	    //     press=XGZP_ReadByte(I2C1, 0x5) << 16;
	    press=XGZP_ReadByte(I2C1, 0x6) << 8;
	    press|=XGZP_ReadByte(I2C1, 0x7) ;

// if (press&0x8000) press |=0xffff0000;  //расширение знака

	    temper=XGZP_ReadByte(I2C1, 0x9) << 8;
	    temper|=XGZP_ReadByte(I2C1, 0xa);
	    Delay(1);

	    if (i2c_error==0)
	    {
	    	CorrConf[1]->rawpress_i2c = press;
	    	CorrConf[1]->temper_i2c = temper*100/256;
//	 CorrConf->press_i2c = ((s32)(temp16)*5000/32767-5000-CorrConf->press_offset);
//	  CorrConf[1]->press_i2c = (((temp16+1024)/60)-500-CorrConf[1]->press_offset)*(1+(CorrConf[1]->kf_press/100));
//	  CorrConf[1]->press_i2c = ( (temp16+1024)/60 -500-CorrConf[1]->press_offset)*(100+CorrConf[1]->kf_press)/100;
	     // CorrConf[1]->press_i2c = (press-CorrConf[1]->press_offset)*(100+CorrConf[1]->kf_press)/100; // для увеличения на К процентов
	   // для увеличения на К процентов
	    	press32=press;
	    	press32-=CorrConf[1]->press_offset;
	    	if (press32>0x7fff) press32 =0x7fff;
	    	if (press32<-0x8000) press32 =-0x8000;
//	  CorrConf[1]->tcomp_offset = (2.5*(CorrConf[0]->temper_base/100 - CorrConf[1]->temper_i2c/100));
//	  CorrConf[1]->tcomp_diap = ( press32 + CorrConf[1]->tcomp_offset ) * (100+CorrConf[1]->kf_press)/100 * (1.65 * (CorrConf[1]->temper_i2c - CorrConf[0]->temper_base)/CorrConf[1]->press_diap)/100;
	    	CorrConf[1]->filter = ((CorrConf[1]->filter*19)+  press32)/20;
//		  CorrConf[1]->filter = ((CorrConf[1]->filter*19)+  ( press32 + CorrConf[1]->tcomp_offset ) * (100+CorrConf[1]->kf_press)/100 + CorrConf[1]->tcomp_diap)/20;
	 //	  CorrConf[1]->filter =   (( press32 + CorrConf[1]->tcomp_offset ) * (100+CorrConf[1]->kf_press)/100 + CorrConf[1]->tcomp_diap);

      // запись значения давления второго канала в регистр
	    	CorrConf[1]->press_i2c=CorrConf[1]->filter*10/16;

	  //	 dev_conf[1].devConfig[0x11]=CorrConf[1]->press_i2c;
//	  CorrConf[1]->press_i2c/=2;
//	dev_conf[1].devConfig[0x11]>>=4;
 //	CorrConf[1]->press_i2c=dev_conf[1].devConfig[0x11];
	//CorrConf[1]->press_i2c = ( press - CorrConf[1]->press_offset + CorrConf[1]->tcomp_offset ) * (100+CorrConf[1]->kf_press)/100;

	    	if ((CorrConf[0]->WorkMode & 1)==0) //режим "шлюз" выключен
	    	{
	    		if (VentConf->set_select == 0) //первая уставка
	    			if (VentConf->fan_on != 0) //вентилятор включен
	    				if (abs(VentConf->cur_flaw - CorrConf[0]->ustav)<(CorrConf[0]->ustav / 5)) //если текущий поток в пределах -20% +20% от уставки - можно выдавать аварию фильтра
	    					if (CorrConf[1]->press_i2c/10 > VentConf->max_pres) VentConf->filter_alarm = 1;
	    					else if (CorrConf[1]->press_i2c/10 < VentConf->min_pres) VentConf->filter_alarm = 2;
	    					else VentConf->filter_alarm = 0;
	    				else VentConf->filter_alarm = 0;  //поток меньше 80% или больше 120% от заданного - нет аварии фильтра
	    			else VentConf->filter_alarm = 0;	//вентилятор выключен - нет аварии фильтра
	    		else VentConf->filter_alarm = 0;	//вторая уставка - нет аварии фильтра
	    	}
	    	else //режим "шлюз"
	    	{
  //тут надо сформировать биты аварии фильтра и вентиляторов
	    	}

	    	CorrConf[1]->i2c_success_count++;
	    	CorrConf[1]->i2c_sens_error = 0; //нет ошибок
	    }

  //			  (temp16- 1024.0) / 6.0 - 5000;

	    else
	    {
	    	CorrConf[1]->i2c_sens_error = i2c_error; //ошибка датчика
	    	i2c_error=0;
	    	I2C1->CR1 |= I2C_CR1_SWRST; //reset
// конфигурируем ножки на выход (иногда слетала конфигурация)
	    	GPIOB->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1 |GPIO_CRL_CNF7_0 | GPIO_CRL_CNF7_1;

	    	if (!((GPIOB->IDR) & (1<<7))) //слейв держит шину
	    	{
	    		while  (!((GPIOB->IDR) & (1<<7)))
	    		{
	    			GPIOB->CRL &= ~ GPIO_CRL_CNF6_1; //шлём клоки, пока слейв не отпуситит шину
	    			Delay(1);
	    			GPIOB->CRL |= GPIO_CRL_CNF6_1;
	    			Delay(1);
	    		}
	    	}

	    	Delay(10);
	    	I2C1->CR1 &= ~I2C_CR1_SWRST; //отпускаем reset
	    	init_I2C(I2C1);	//инит i2c после ресета
	    }

	    //отключаемся от 2го датчика
	    GPIOB->ODR |= GPIO_ODR_ODR6 | GPIO_ODR_ODR7;
	    GPIOB->CRL &= ~GPIO_CRL_CNF6_0 & ~GPIO_CRL_MODE6_1 & ~GPIO_CRL_MODE6_0 & ~GPIO_CRL_CNF7_0 & ~GPIO_CRL_MODE7_1 & ~GPIO_CRL_MODE7_0;
	    //подключаемся к 1му датчику
	    GPIO_PinRemapConfig( GPIO_Remap_I2C1 , ENABLE );
	    GPIOB->CRH |= GPIO_CRH_CNF8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0 | GPIO_CRH_CNF9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;
	    GPIOB->ODR &= ~GPIO_ODR_ODR8 & ~GPIO_ODR_ODR9;


// ------------------------ чтение PCA9534 -------------------------------------------------------------

//	    CorrConf[0]->PCA9534_0 = ~SW_I2C_ReadControl_8Bit(SW_I2C2,0x40,0);
  // CorrConf[0]->PCA9534_2 = SW_I2C_ReadControl_8Bit(SW_I2C2,0x40,2);

	    // запись в PCA9534 новых значений
//	    CorrConf[0]->sw_i2c_connection = SW_I2C_WriteControl_8Bit(SW_I2C2,0x40,3,~(CorrConf[0]->PCA9534_3));
//	    SW_I2C_WriteControl_8Bit(SW_I2C2,0x40,1,(CorrConf[0]->PCA9534_1));

// I2C_StartTransmission(I2C2,I2C_Direction_Transmitter,0x40);
// I2C_WriteData(I2C2,1);
// I2C_WriteData(I2C2,Count_time);
// I2C_GenerateSTOP(I2C2, ENABLE);
	    Count_time++;
	    RxBuffer[1]=Count_time & 0xff;
	    VentConf->LightOn=RxBuffer[0];


// ----------------------- читаем с hx710 температуру и давление -------------------------------------

	    GPIOB->ODR&=~(GPIO_ODR_ODR15);    //SCK = 0
	    Count=0;
	    if (!(GPIOB->IDR&(GPIO_IDR_IDR14)))
	    {
	    	for (i=0;i<24;i++)
	    	{
	    		hx711_clkpulse (); //импульс на clk
	    		Count<<=1;
	    		if(GPIOB->IDR&(GPIO_IDR_IDR14)) Count++;
	    	}
	    	hx711_clkpulse (); //25й импульс
	    	hx711_clkpulse (); //26й импульс
	    	if (hx710_phase==0) //была фаза давления
	    	{
	    		hx710_phase=1; //переключаемся на температуру
	    		VentConf->HX711_press=Count>>8;
	    	}
	    	else
	    	{
	    		VentConf->HX711_temper=(Count-2560000)>>7;
	    		hx710_phase=0; //переключаемся на давление
	    		hx711_clkpulse (); //27й импульс
	    	}
	    }

// **************** логика режима "шлюз" ********************************************************

	    // обработка регистра debug
	    if (VentConf->debug ==1 ) { CorrConf[0]->WorkMode &= ~0x01; } // бит шлюза сбрасываем
	    else { CorrConf[0]->WorkMode |= 0x01; } // бит шлюза устанавливаем

	    u16 LightOnMoment, FanOnMoment, CheckOnMoment;

	    if (CorrConf[0]->WorkMode & 1) //режим "шлюз"
	    {
	    	VentConf->fan_on=0; //выкл регулятора

	    	switch (GateWayState)
	    	{
	    	case 0: //ждём открытия любой двери
	    		TIM3->CCR3=0;       //выкл вентилятора
	    		VentConf->fan_power=0;   //
	    		VentConf->UV_on=0; //выкл УФ
	    		CorrConf[0]->PCA9534_1 &= ~0xf0; //разблокировка дверей,  выкл светофор, выкл пищалки
	    		if (SecondCounter-LightOnMoment>CorrConf[0]->LightTime)	  VentConf->LightOn=0; //выкл свет
	    		if (CorrConf[0]->PCA9534_0 & 1) //открыта дверь 1 (грязная)
	    		{
	    			GateWayState=1;
	    			CorrConf[0]->PCA9534_1 |= 0x70; //блокировка противоположной двери и вкл светофор, пищалка
	    			VentConf->LightOn=1; //вкл свет
	    		};
	    		if (CorrConf[0]->PCA9534_0 & 2) //открыта дверь 2 (чистая)
	    		{
	    			GateWayState=2;
	    			CorrConf[0]->PCA9534_1 |= 0xb0; //блокировка противоположной двери, вкл светофор, пищалка
	    			VentConf->LightOn=1; //вкл свет
	    		};
	    		break;

	    	case 1: //ждём закрытия двери 1
	    		if ((CorrConf[0]->PCA9534_0 & 3)==0) //обе двери закрыты
	    		{
	    			CorrConf[0]->PCA9534_1 &= ~0x10; // выкл пищалки
	    			TIM3->CCR3=CorrConf[0]->InFanPower*65535/10000;       //вкл вентилятора
	    			VentConf->fan_power=CorrConf[0]->InFanPower;   //
	    			FanOnMoment=SecondCounter; //взводим таймер выключения вентилятора
	    			if ((CorrConf[0]->WorkMode & 4)==0)  //уф включен
	    			{
	    				VentConf->UV_on=1; //вкл УФ
	    			};
	    			GateWayState=3;
	    			CheckOnMoment =SecondCounter; // взводим таймер проверки фильтра
	    		}
	    		break;

	    	case 2: //ждём закрытия двери 2
	    		if ((CorrConf[0]->PCA9534_0 & 3)==0) //обе двери закрыты
	    		{
	    			CorrConf[0]->PCA9534_1 &= ~0x10; // выкл пищалки
	    			if (CorrConf[0]->WorkMode & 2)  //двусторонний шлюз
	    			{
	    				TIM3->CCR3=CorrConf[0]->InFanPower*65535/10000;       //вкл вентилятора
	    				VentConf->fan_power=CorrConf[0]->InFanPower;   //
	    				FanOnMoment=SecondCounter; //взводим таймер выключения вентилятора
	    				//VentConf->UV_on=1; //вкл УФ
	    				if  ((CorrConf[0]->WorkMode & 4)==0)   //уф включен
	    				{
	    					VentConf->UV_on=1; //вкл УФ
	    				};
	    				GateWayState=3;
	    				CheckOnMoment =SecondCounter; // взводим таймер проверки фильтра
	    			}
	    			else
	    			{
	    				GateWayState=4;
	    			}
	    		}
	    		break;

	    	case 3:  //ждём окончания цикла обработки
	    		CorrConf[0]->PCA9534_1 |= 0xc0; //блокировка двух дверей

	    		//мигаем светофором
	    		if (mSecondCounter>500) CorrConf[0]->PCA9534_1 &= ~0x20; //выкл светофор
	    		else  CorrConf[0]->PCA9534_1 |= 0x20; //вкл светофор

	    		//если время обработки закончилось переходим в начальное состояние
	    		if (SecondCounter-FanOnMoment > CorrConf[0]->BlowTime)
	    		{
	    			GateWayState=4;
	    		}

	    		//если время ожидания проверки закончилось - начинаем проверку значений
	    		if (SecondCounter-CheckOnMoment > CorrConf[0]->CheckTime)
	    		{
	    			// Проверка аварии 1 мотора
	    			if (CorrConf[0]->FanSpeed < 5 )
	    			{
	    				VentConf->fan_alarm |= 1;
	    			}
	    			else
	    				VentConf->fan_alarm &= ~1;

	    			// Проверка аварии 2 мотора
	    			if (CorrConf[1]->FanSpeed < 5 )
	    			{
	    				VentConf->fan_alarm |= 2;
	    			}
	    			else
	    				VentConf->fan_alarm &= ~2;

	    			// Проверка макс давления на фильтре 1
	    			if (VentConf->max_pres*10 < CorrConf[0]->press_i2c )
	    			{
	    				VentConf->filter_alarm |= 1;
	    			}
	    			else
	    				VentConf->filter_alarm &= ~1;  //сброс ошибки засора фильтра


	    			// Проверка макс давления на 2 фильтре
	    			if (VentConf->max_pres*10 < CorrConf[1]->press_i2c )
	    			{
	    				VentConf->filter_alarm |= 4;
	    			}
	    			else
	    				VentConf->filter_alarm &= ~4;  //сброс ошибки засора фильтра


	    			// Проверка мин давления на фильтре 1
	    			if (VentConf->min_pres*10 > CorrConf[0]->press_i2c )
	    			{
	    				VentConf->filter_alarm |= 2;
	    				GateWayState=4; // переходим с ошибкой в начальное состояние
	    			}
	    			else
	    				VentConf->filter_alarm &= ~2;  //сброс ошибки порыва первого фильтра

	    			// Проверка мин давления на 2 фильтре
	    			if (VentConf->min_pres*10 > CorrConf[1]->press_i2c )
	    			{
	    				VentConf->filter_alarm |= 8;
	    				GateWayState=4; // переходим с ошибкой в начальное состояние
	    			}
	    			else
	    				VentConf->filter_alarm &= ~8;  //сброс ошибки порыва вротого фильтра

	    		}


	    		break;

	    	default:
	    		LightOnMoment = SecondCounter; //взводим световой таймер
	    		GateWayState=0;
	    		break;
	    	} //end switch
	    } //режим шлюз

	    VentConf->debug2 = stop_count;
	    //меняем состояние светодиода
	    GPIO_WriteBit(GPIOC, GPIO_Pin_15, !GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15));

    } //end While
} // end main
