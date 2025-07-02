#include "delay.h"

void delay_us(uint32_t us)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable counter

    //将 CPU 主频（Hz）→ 转换为微秒级单位的周期数；
    /*假设 MCU 的频率是 SystemCoreClock = 72,000,000（72MHz）：
    每秒：72,000,000 个周期
    每微秒：72,000,000 / 1,000,000 = 72 个周期*/
    uint32_t cycles = us * (SystemCoreClock / 1000000); 
    
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles);
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        delay_us(1000);
    }
}
