#include "hardware_iic.h"
unsigned char IIC_ReadByte(unsigned char Salve_Adress,unsigned char Reg_Address)
{
	return hardware_IIC_ReadByte(Salve_Adress,Reg_Address);
}
unsigned char IIC_ReadBytes(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *Result,unsigned char len)
{
	return hardware_IIC_ReadBytes(Salve_Adress,Reg_Address,Result,len);
}
unsigned char IIC_WriteByte(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char data)
{
	return hardware_IIC_WirteByte(Salve_Adress,Reg_Address,data);
}
unsigned char IIC_WriteBytes(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *data,unsigned char len)
{
	return hardware_IIC_WirteBytes(Salve_Adress,Reg_Address,data,len);
}
unsigned char Ping(void)
{
	unsigned char dat;
	IIC_ReadBytes(Color_Adress<<1,PING,&dat,1);
	if(dat==PING_OK)
	{
			return 0;
	}	
	else return 1;
}
unsigned char IIC_Get_Error(void)
{
	unsigned char dat;
	IIC_ReadBytes(Color_Adress<<1,Error,&dat,1);
	return dat;
}
unsigned char IIC_Get_RGB(unsigned char * Result,unsigned char len)
{
	if(IIC_ReadBytes(Color_Adress<<1,RGB_Reg,Result,len))return 1;
	else return 0;
}
unsigned char IIC_Get_HSL(unsigned char * Result,unsigned char len)
{
	if(IIC_ReadBytes(Color_Adress<<1,HSL_Reg,Result,len))return 1;
	else return 0;
}

