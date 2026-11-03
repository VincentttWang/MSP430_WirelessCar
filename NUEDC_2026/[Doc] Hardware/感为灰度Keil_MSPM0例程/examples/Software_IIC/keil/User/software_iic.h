#include "ti_msp_dl_config.h"
#include "gw_color_sensor.h"
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
uint8_t IIC_WriteBytes(uint8_t Salve_Address, uint8_t Reg_Address,
                           uint8_t *data, uint8_t len);
uint8_t IIC_WriteByte(uint8_t Salve_Address, uint8_t Reg_Address, 
                          uint8_t data);
void i2c_reset();

