#include "ti_msp_dl_config.h"
#include "Time.h"
#include "stdio.h"
#include "Uart.h"
#include "string.h"
#include "software_iic.h"
/*************************************软件IIC测试Demo**********************************************/
/*****************芯片型号 MSPM0G3507 SystemConfig 标准库******************************************/
/*****************引脚 SDA PA1 SCL PA0  ***********************************************************/
/*****************串口 Tx PA10 Rx PA11 ************************************************************/
/*****************请插上SCL SDA跳线帽，以便上拉总线，如果不插可能无法通讯*******************/
/*****************不要插AD0 AD1 PULL********************************************************/
/*****************传感器供电需要4.5-5V电压稳定供电，否则可能无法正常使用********************/
/*****************保证单片机和传感器共地，如果不共地无法通讯********************************/
/*****************这只是示例，示例不售后，一定要去看文档，这十分重要************************/
/***************************************软件IIC测试Demo********************************************/
extern uint8_t IIC_ReadBytes(uint8_t Salve_Address, uint8_t Reg_Address, 
                          uint8_t *Result, uint8_t len);
/*
全局变量
*/
uint8_t rx_buff[256]={0};//uart1打印用的，不用管，不要动
uint8_t IIC_write_buff[10]={0};//IIC写数据用的buff
uint8_t Anolog[8]={0};//1-8路模拟量数据
uint8_t Normal[8]={0};//归一化后的1-8路模拟量数据
uint8_t Digtal;
int main(void)
{
    SYSCFG_DL_init();
		sprintf((char *)rx_buff,"hello_world!\r\n");
  	/* 广播并重置软件地址为0b10011 即0x4c*/
		i2c_reset();
		uart0_send_string((char *)rx_buff);
		memset(rx_buff,0,256);	
		while(Ping())
		{
			delay_ms(1);
			sprintf((char *)rx_buff,"Ping Faild Try Again!\r\n");
			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
		}
		sprintf((char *)rx_buff,"Ping Succseful!\r\n");
		uart0_send_string((char *)rx_buff);
		memset(rx_buff,0,256);
		while (1) {
			/*数字量*/
			/*数字量数据和1-8号指示灯相同，首先需要校准，才能正常读取*/
			/*数字量读取方法1：比较费时，但易于理解，其他更高效方法请看文档*/
			IIC_ReadBytes(GW_GRAY_ADDR_DEF << 1/* 默认不插AD0 AD1地址0x4c */, GW_GRAY_DIGITAL_MODE/* 0xDD */, &Digtal/*数据存放在Digtal里*/, 1 /*只读1个数据*/);
			
			/*模拟量*/
			/*连续通道模拟量读取与校准无关*/
			/*在同一个白场下，模拟量数据会有所差异，这是正常现象，如果您想要数据相同，请校准后打开归一化*/
			/*连续通道模拟量读取方法1：比较费时，但易于理解，其他更高效方法请看文档*/
			IIC_ReadBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE/* 0xB0 */, &Anolog[0] /*数据存放在Anolog里*/, 8 /*读8个数据*/);
			
			
			/*归一化*/
			/*归一化的意思是，使所有探头，在同一个白色或者黑色下，数据是一致的*/
			/*这个与校准是有关的，他是通过校准数据给模拟量进行软件处理归一化的*/
			IIC_write_buff[0]=GW_GRAY_ANALOG_NORMALIZE;//归一化使能寄存器，掉电不存储
			IIC_write_buff[1]=0xff;//全通道开启
			IIC_WriteBytes(GW_GRAY_ADDR_DEF << 1,GW_GRAY_ANALOG_NORMALIZE ,&IIC_write_buff[1]/*命令+数据*/, 2 /*写入两个数据*/);
			delay_ms(10);//设置完，需要等上一会。stm8的运算速度没stm32快，等一下，让传感器把数据刷新一下。
			IIC_ReadBytes(GW_GRAY_ADDR_DEF << 1, GW_GRAY_ANALOG_MODE, &Normal[0]/*打开归一化后读到的数据存在Normalize里*/ , 8 );//跟上面连续通道模拟量读取一样
			IIC_write_buff[0]=GW_GRAY_ANALOG_NORMALIZE;
			IIC_write_buff[1]=0x00;//全通道关闭
			IIC_WriteBytes(GW_GRAY_ADDR_DEF << 1,GW_GRAY_ANALOG_NORMALIZE,&IIC_write_buff[1]/*命令+数据*/, 2 /*写入两个数据*/);//为了while(1)循环继续，当然是要关掉归一化的。
					
			delay_ms(10);
			/*打印数据*/
			sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			
			sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			
			sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			delay_ms(1);
		}
}