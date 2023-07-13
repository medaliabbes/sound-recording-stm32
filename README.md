# Sound recorder
## STM32F103 + SDcard + MAX4466 

this project demonstrate the use of the combination Timer ADC DMA to capture sound without the intervention of the CPU .
first we should choose the sampling frequency , witch is 8000Hz for this project (the maximum for F103) ,then set the timer interrupt frequency to the sampling frequency ,
in the ADC configuration set ADC trigger to Timer trigger witch mean every time a timer interrupt occure that will trigger an adc conversion ,when the DMA is enabled the result of the conversion will transmitted via DMA to a buffer, all of this happen withou CPU .

## Dependency 
-  [stm32f10x-stdperiph-lib](#https://github.com/wajatimur/stm32f10x-stdperiph-lib)
-  [FAT32](#http://elm-chan.org/fsw/ff/00index_e.html)
