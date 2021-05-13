#ifndef __MAIN__
#define __MAIN__


#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "system_stm32f10x.h"
#include "stm32f10x_exti.h"
#include "ff.h"
#include "fatfs.h"
#include "wave.h"

void gpio_init(void);
void timer_init(void);
void spi_init(void);
void dma_init(void);
void adc_init(void);
void clock_enable(void);
void interrupt_init(void);
void incriment_tick(void);
uint32_t get_tick(void);
void delay_ms(uint32_t delay);
void SDTimer_Handler(void);
void timer3_start();
void adc_start_trig_conv(void);
void dma_start(void);

#endif