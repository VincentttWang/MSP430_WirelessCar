#include "ti_msp_dl_config.h"
#include "Time.h"
#include "stdio.h"
#include "Uart.h"
#include "string.h"
/***************************************串行测试_Demo**********************************************/
/*****************芯片型号 MSPM0G3507 SystemConfig 标准库******************************************/
/*****************引脚 DAT PB8 CLK PB9  ***********************************************************/
/*****************串口 Tx PA10 Rx PA11 ************************************************************/
/***************************************串行测试_Demo**********************************************/
unsigned char Digtal;
unsigned char rx_buff[256]={0};
uint8_t gw_gray_serial_read()
{
	uint8_t ret = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		/* 输出时钟下降沿 */
		DL_GPIO_clearPins(GPIOB, Serial_CLK_PIN);
		delay_us(2);
		//避免GPIO翻转过快导致反应不及时
		ret |= (DL_GPIO_readPins(GPIOB, Serial_DAT_PIN)==0?0:1) << i;

		/* 输出时钟上升沿,让传感器更新数据*/
		DL_GPIO_setPins(GPIOB, Serial_CLK_PIN);
	
		/* 延迟需要在5us左右 */
		delay_us(5);
	}
	
	return ret;
}
int main(void)
{
    SYSCFG_DL_init();
		sprintf((char *)rx_buff,"hello_world!\r\n");
		uart0_send_string((char *)rx_buff);
		memset(rx_buff,0,256);	
		while (1) {
		Digtal=gw_gray_serial_read();
		sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
		uart0_send_string((char *)rx_buff);
		memset(rx_buff,0,256);
			delay_ms(1);
		}
}