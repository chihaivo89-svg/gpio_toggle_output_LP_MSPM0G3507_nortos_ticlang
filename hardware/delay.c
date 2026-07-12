#include "ti_msp_dl_config.h"
#include "delay.h"

void Delay_ms(volatile uint32_t ms)
{
    while(ms--)
        delay_cycles(CPUCLK_FREQ/1000);

}

void Delay_us(volatile uint32_t us)
{
    while(us--)
        delay_cycles(CPUCLK_FREQ/1000000);

}