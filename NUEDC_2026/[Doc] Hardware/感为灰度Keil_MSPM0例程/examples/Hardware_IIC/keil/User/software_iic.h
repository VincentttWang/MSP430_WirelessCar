#include "ti_msp_dl_config.h"
#include "gw_grayscale_sensor.h"
#include "Time.h"

#define SDA_PIN Software_iic_SDA_PIN
#define SDA_PORT Software_iic_PORT
#define SCL_PIN Software_iic_SCL_PIN
#define SCL_PORT Software_iic_PORT

/* 基本I2C操作宏 */
#define SDA_HIGH() DL_GPIO_setPins(SDA_PORT, SDA_PIN)
#define SDA_LOW()  DL_GPIO_clearPins(SDA_PORT, SDA_PIN)
#define SCL_HIGH() DL_GPIO_setPins(SCL_PORT, SCL_PIN)
#define SCL_LOW()  DL_GPIO_clearPins(SCL_PORT, SCL_PIN)
#define READ_SDA() DL_GPIO_readPins(SDA_PORT, SDA_PIN)

unsigned char Ping(void);
unsigned char IIC_Get_Digtal(void);
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len);
unsigned char IIC_Get_Single_Anolog(unsigned char Channel);
unsigned char IIC_Get_Normalize(unsigned char * Result,unsigned char len);
unsigned short IIC_Get_Offset(void );

