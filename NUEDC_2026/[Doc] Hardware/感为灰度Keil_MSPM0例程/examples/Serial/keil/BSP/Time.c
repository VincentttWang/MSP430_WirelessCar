#include "Time.h"
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    ticks = __us * (80000000 / 1000000);//根据自己主频来我这里是80Mhz

    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            if (tcnt >= ticks)
                break;
        }
    }
}
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}