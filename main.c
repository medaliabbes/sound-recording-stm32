#include "main.h"

volatile uint32_t uTick = 0;
volatile uint8_t FatFsCnt = 0;
volatile uint8_t Timer1, Timer2;
uint16_t count = 0,count1 = 0;

static void fill_audio_buff(PCM16_stereo_t * audio,uint16_t * tab,uint8_t start,uint8_t end) ;

FATFS fs; 
FIL fil;

uint16_t dma_buffer[102];
PCM16_stereo_t audio_buffer[100];
uint8_t ht_state = 0,tc_state = 0,recording_on = 0;
uint32_t time = 0;

int main(void)
{
    SystemInit(); // set clock to 72Mhz
    /* timers clock = 72Mhz if the hclk = 72Mhz*/
    SysTick_Config(71999);//(72Mhz/1000Hz) -1 ====> system interrupt every 1ms
    clock_enable();
    gpio_init();
    spi_init();
    timer_init();
    
    MX_FATFS_Init();
    f_mount(&fs, "",0);
    f_open(&fil,"rec.wav", FA_OPEN_ALWAYS|FA_WRITE);
    uint8_t duration = 10;
    wave_write_header(&fil,SAMPLE_RATE,duration*SAMPLE_RATE);

    
    dma_init();
    dma_start();
    
    adc_init();
    adc_start_trig_conv();//this fonction enable adc ,dma,trigger conv 

    timer3_start();
    interrupt_init();

    GPIO_WriteBit(GPIOC,GPIO_Pin_13,1);//led off
    
    recording_on = 1;
    UINT bw;
    while(1)
    {
       if(recording_on == 1)
       {
            GPIO_WriteBit(GPIOC,GPIO_Pin_13,0);//led on
            if(ht_state == 1)
            {
                fill_audio_buff(audio_buffer,dma_buffer,0,50);
                f_write(&fil,&audio_buffer,50*sizeof(PCM16_stereo_t),&bw);
                ht_state = 0;
            }
            if(tc_state == 1)
            {
                fill_audio_buff(audio_buffer,dma_buffer,50,100);
                f_write(&fil,&audio_buffer[50],50*sizeof(PCM16_stereo_t),&bw);
                tc_state = 0;
            }
            else
            {
                //nop();
            }
       }    

        else if(recording_on == 0 && time >0)
        {   
            time = time / 1000;//time in ms
            time = time / 10;//time in seconds
            fil.fptr = 0;
            wave_write_header(&fil,SAMPLE_RATE,time*SAMPLE_RATE);
            f_close(&fil);
            NVIC_DisableIRQ(DMA1_Channel1_IRQn);
            NVIC_DisableIRQ(EXTI1_IRQn);
            goto end;
        } 
     }

    end :
        GPIO_WriteBit(GPIOC,GPIO_Pin_13,!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13));
        delay_ms(200);
        goto end;
    
    loop :
        goto loop;

}

static void fill_audio_buff(PCM16_stereo_t * audio,uint16_t * tab,uint8_t start,uint8_t end)
{
    for(uint8_t i = start;i<end;i++)
    {
        /*
            this shift operation is for  amplification 
        */
        audio[i].left = tab[i]<<3;
        audio[i].right = tab[i]<<3;
    }
} 


void gpio_init(void)
{

    /*led init*/
    GPIO_WriteBit(GPIOC,GPIO_Pin_14,1);
    GPIO_InitTypeDef led; 
    led.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
    led.GPIO_Speed = GPIO_Speed_10MHz;
    led.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC,&led);

    /*******spi1 pins***/
    GPIO_InitTypeDef GPIO_InitStructure;
    // spi mosi and clock pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOA, &GPIO_InitStructure) ;
    //spi miso pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    // output push-pull mode
    GPIO_Init(GPIOA, &GPIO_InitStructure) ; 
    
    //slave select pinA4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOA, &GPIO_InitStructure) ; 

    /*********adc pin ****/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN ;//analog
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    /***********interrupt pin**************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOA, &GPIO_InitStructure) ;
}

void timer_init(void)
{
    TIM_TimeBaseInitTypeDef tim3;
    NVIC_InitTypeDef tim3_interrupt;

    tim3.TIM_Prescaler = 71;//timer clock = 1Mhz clock period = 1µs
    tim3.TIM_CounterMode = TIM_CounterMode_Up ;
    tim3.TIM_Period = 124 ;// frequency  of trigger = 8Khz 
    tim3.TIM_ClockDivision = TIM_CKD_DIV1 ;//no division
    TIM_TimeBaseInit(TIM3,&tim3);

    TIM_InternalClockConfig(TIM3);//timer3 clock internal

    TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);
    
    TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

    /*TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
    tim3_interrupt.NVIC_IRQChannel = TIM3_IRQn;
    tim3_interrupt.NVIC_IRQChannelPreemptionPriority = 1;
    tim3_interrupt.NVIC_IRQChannelSubPriority = 1;
    tim3_interrupt.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&tim3_interrupt);*/
}

void timer3_start(void)
{
    TIM_Cmd(TIM3,ENABLE);
}

void spi_init(void)
{
    SPI_InitTypeDef spi1;
	spi1.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi1.SPI_DataSize = SPI_DataSize_8b;
    spi1.SPI_Mode = SPI_Mode_Master ;
    spi1.SPI_CPOL = SPI_CPOL_Low;
    spi1.SPI_CPHA = SPI_CPHA_1Edge;
    spi1.SPI_NSS = SPI_NSS_Soft;
    spi1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    spi1.SPI_FirstBit = SPI_FirstBit_MSB;
    spi1.SPI_CRCPolynomial = 10;
    SPI_Init(SPI1,&spi1);
    SPI_Cmd(SPI1,ENABLE); 
}

void dma_init(void)
{
    DMA_InitTypeDef dma;
    //NVIC_InitTypeDef dma_interrupt;

    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr = (uint32_t)dma_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC ;
    dma.DMA_BufferSize = 100;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable ;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord ;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High ;
    dma.DMA_M2M = DMA_M2M_Disable ;
    DMA_Init(DMA1_Channel1,&dma);
    DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
    DMA_ITConfig(DMA1_Channel1,DMA_IT_HT,ENABLE);
}

void dma_start(void)
{
    DMA_Cmd(DMA1_Channel1,ENABLE);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void adc_init(void)
{
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//adc clk 12MHZ
    ADC_InitTypeDef adc;

    adc.ADC_Mode = ADC_Mode_Independent ;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    adc.ADC_DataAlign =  ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1,&adc);

    ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_1Cycles5);//convertion time = 0.0016ms

    //calibration 
    ADC_StartCalibration(ADC1);//calibration is necessairy
    while(ADC_GetCalibrationStatus(ADC1)==RESET);//wait calibration flag to set
    
}

void adc_start_trig_conv(void)
{
    ADC_Cmd(ADC1,ENABLE);
    ADC_DMACmd(ADC1,ENABLE);
    ADC_ExternalTrigConvCmd(ADC1,ENABLE);
}

void clock_enable(void)
{
    /****enable clock for led GPIOC********/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    /*****enable clock for spi pins and adc pin GPIOA*****/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    /*****enable clock for timer 3*****/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    /*****enable spi1 clock********/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
    /******enable clock for adc*******/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    /******enable clock for dma1********/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
}

void interrupt_init(void)
{
    EXTI_InitTypeDef exti;

    exti.EXTI_Line = EXTI_Line1 ; 
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising ;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void SDTimer_Handler(void)
{
  if(Timer1 > 0)
    Timer1--;

  if(Timer2 > 0)
    Timer2--;
}

void incriment_tick(void)
{
    uTick++;
}

void delay_ms(uint32_t delay)
{
    uint32_t st = get_tick();
    while(get_tick()-st<delay);
}

uint32_t get_tick(void)
{
    return uTick;
}

/**system interrupt handler**/
void SysTick_Handler(void)
{
    FatFsCnt++;
	  if(FatFsCnt >= 10)
	  {
	    FatFsCnt = 0;
	    SDTimer_Handler();
	  }
   incriment_tick(); 
}

void DMA1_Channel1_IRQHandler(void)
{
    if(recording_on == 1)
    {
        if(DMA_GetITStatus(DMA1_IT_TC1))//the same as DMA_GetFlagStatus(DMA1_FLAG_TC1)
        {
            tc_state = 1;
            count++;
            time = time +125;
            //GPIO_WriteBit(GPIOC,GPIO_Pin_13,!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13));
            DMA_ClearITPendingBit(DMA1_IT_TC1);
        }

        else if(DMA_GetITStatus(DMA1_IT_HT1))//the same as DMA_GetFlagStatus(DMA1_FLAG_TC1)
        {
            ht_state = 1;
            DMA_ClearITPendingBit(DMA1_IT_HT1);
        } 
    }  
}

void EXTI1_IRQHandler(void)
{
    if(EXTI_GetFlagStatus(EXTI_Line1))
    {
        if(recording_on == 0)
        {
            recording_on = 1;
        }
        else if(recording_on == 1)
        {
            recording_on = 0;
        }
        EXTI_ClearFlag(EXTI_Line1);
    }

}
