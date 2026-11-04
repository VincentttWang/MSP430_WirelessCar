/**
 * @file  oled.c
 * @brief SSD1306 OLED 硬件 I2C 显示实现。
 */

#include "oled.h"
#include "oled_font.h"
#include "oled_font_cn.h"
#include "oled_font_arial.h"
#include "bsp_time.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define OLED_CMD  0U
#define OLED_DATA 1U
#define OLED_I2C_ADDR_7BIT 0x3CU
#define OLED_I2C_TIMEOUT   20000U
/* OLED 插拔检测周期：屏幕拔掉后 IO 悬空（模块自带的 4.7k 上拉跟着 VCC 一起没了），
 * BUSBSY 会一直挂住；每 2s 只花一条地址字节探一次 ACK，探不到就跳过刷屏，
 * 免得整机被几十条 OLED 事务的超时拖到遥测都发不出去。 */
#define OLED_PRESENCE_PERIOD_MS  2000U

static bool OLED_I2C_WaitIdle(void);
static void OLED_I2C_BusRecover(void);
static bool OLED_ProbeAck(void);
static bool OLED_PresenceCheck(void);
static bool s_oled_online = true;
static uint32_t s_oled_probe_ms = 0U;
static bool OLED_I2C_Write(const uint8_t *data, uint16_t len);
static void OLED_Write_Bytes(uint8_t control, const uint8_t *data, uint16_t len);
static void OLED_WR_Byte(uint8_t dat, uint8_t mode);
static void OLED_Write_ContinuousCmd(const uint8_t *cmds, uint8_t len);
static void OLED_Write_ContinuousData(const uint8_t *data, uint16_t len);
static void OLED_Set_Pos(uint8_t x, uint8_t y);
static uint32_t oled_pow(uint8_t m, uint8_t n);

static void OLED_DelayMs(uint32_t ms){
    BSP_DelayMs(ms);
}

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127
 
 
//反显函数
void OLED_ColorTurn(uint8_t i){
    if(i==0){
        OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
    }
    if(i==1){
        OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
    }
}
 
////屏幕旋转180度
//void OLED_DisplayTurn(uint8_t i)
//{
//if(i==0)
//    {
//        OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
//        OLED_WR_Byte(0xA1,OLED_CMD);
//    }
//    if(i==1)
//    {
//        OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
//        OLED_WR_Byte(0xA0,OLED_CMD);
//    }
//}
/* 只发一个地址字节看 ACK：true = 屏在线。
 *
 * 【判据要三条都满足】之前只看 ERROR 位是不够的：总线被拉死（BUSBSY）时
 * START 根本发不出去，控制器既不置 BUSY 也不置 ERROR，于是"什么都不做"
 * 被误判成"在线"，写函数继续逐条等超时 —— 整机就被拖垮了。
 *   1) 起始前总线不能是 BUSBSY（悬空被拉低 = 屏没供电/没插）；
 *   2) 发完 START 后 BUSY 必须真的置起过（否则等于没发出去）；
 *   3) 收尾 BUSY 清零且无 ERROR（有 ACK）。
 */
static bool OLED_ProbeAck(void){
    uint32_t timeout = OLED_I2C_TIMEOUT;
    uint32_t status;

    while (((status = DL_I2C_getControllerStatus(OLED_INST))
            & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            return false;
        }
    }

    /* 悬空/被拉死的总线：屏不在，别发，也别让上层再等 */
    if ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U){
        return false;
    }

    DL_I2C_resetControllerTransfer(OLED_INST);
    DL_I2C_flushControllerTXFIFO(OLED_INST);
    DL_I2C_transmitControllerData(OLED_INST, 0x00U);
    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    /* BUSY 必须真的置起来，否则说明 START 没发出去（例如又被 BUSBSY 挡住） */
    timeout = 2000U;
    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(OLED_INST);
            return false;
        }
    }

    timeout = OLED_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(OLED_INST);
            return false;
        }
    }

    if ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(OLED_INST);
        return false;
    }
    return true;
}

/* 每 2s 最多探一次；离线期间完全跳过刷屏，在线期间也顺便确认没被拔掉 */
static bool OLED_PresenceCheck(void){
    uint32_t now = BSP_Time_GetMs();

    if ((now - s_oled_probe_ms) < OLED_PRESENCE_PERIOD_MS){
        return s_oled_online;
    }
    s_oled_probe_ms = now;

    if (!s_oled_online){
        /* 离线时先把悬空总线补时钟拉回空闲，再探一次；插回去就能自动恢复 */
        if ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U){
            OLED_I2C_BusRecover();
        }
    }

    bool was_online = s_oled_online;
    s_oled_online = OLED_ProbeAck();

    /* 离线 → 在线的这一拍必须补一次初始化：SSD1306 是重新上电的，
     * 电荷泵/显示开关都还是关的，只送显存它照样黑屏。 */
    if (!was_online && s_oled_online){
        OLED_Init();
    }
    return s_oled_online;
}

/* ═══════════════ I2C1 总线恢复（OLED 把 SDA 拉死时自愈） ═══════════════
 * 症状：I2C1 的 MSR = 0x60（BUSBSY | IDLE）——控制器自己空闲，但总线被拉低，
 *      之后每一次 START 都发不出去，OLED 停止刷新，主循环被逐个超时拖到近乎停摆。
 * 原因：主机在字节传输中途被复位/掉电，SSD1306 还停在"回 ACK 时拉低 SDA"的状态，
 *      必须由主机补 9 个 SCL 时钟 + 一个手工 STOP 它才松手。
 * 做法：临时把 SCL/SDA 从 I2C1 抢成 GPIO，用"高阻 = 释放 / 输出低 = 拉低"
 *      模拟开漏，补完时钟后把复用功能和 HiZ 原样恢复。
 */
static void OLED_I2C_BusRecover(void){
    GPIO_Regs *port = GPIO_OLED_SCL_PORT;
    uint32_t   pins = (uint32_t)GPIO_OLED_SCL_PIN | (uint32_t)GPIO_OLED_SDA_PIN;

    DL_I2C_disableController(OLED_INST);

    /* 抢成 GPIO 输出；先把输出锁存设成 0，再决定是否驱动（DOE=0 即高阻释放） */
    DL_GPIO_initDigitalOutputFeatures(GPIO_OLED_IOMUX_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_initDigitalOutputFeatures(GPIO_OLED_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_clearPins(port, pins);          /* DOUT = 0，等会儿驱动就是拉低 */

    /* 9 个时钟：把从机卡住的那一位（含 ACK）全部推出去 */
    for (uint8_t i = 0U; i < 9U; i++){
        DL_GPIO_disableOutput(port, GPIO_OLED_SCL_PIN);      /* SCL 释放 → 外部上拉为高 */
        DL_Common_delayCycles(160U);                         /* ~5us @32MHz */
        DL_GPIO_enableOutput(port, GPIO_OLED_SCL_PIN);       /* SCL 拉低 */
        DL_Common_delayCycles(160U);
    }

    /* 手工 STOP：SCL 高电平期间 SDA 由低变高 */
    DL_GPIO_enableOutput(port, GPIO_OLED_SDA_PIN);
    DL_Common_delayCycles(160U);
    DL_GPIO_disableOutput(port, GPIO_OLED_SCL_PIN);
    DL_Common_delayCycles(160U);
    DL_GPIO_disableOutput(port, GPIO_OLED_SDA_PIN);
    DL_Common_delayCycles(320U);

    /* 交回 I2C1：复用功能 + HiZ（与 SysConfig 生成的配置保持一致） */
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_IOMUX_SDA,
        GPIO_OLED_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_IOMUX_SCL,
        GPIO_OLED_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SCL);

    DL_I2C_resetControllerTransfer(OLED_INST);
    DL_I2C_enableController(OLED_INST);
}

static bool OLED_I2C_WaitIdle(void){
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(OLED_INST);
            return false;
        }
    }

    /* 控制器空闲但 BUSBSY 还挂着 = 从机把 SDA 拉死，补时钟自愈（只在真卡死时走） */
    if (s_oled_online &&
        (DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U){
        OLED_I2C_BusRecover();
    }

    return true;
}

/* 使用阻塞轮询完成一次 I2C 写事务，调用方负责保证 data 在事务期间有效。 */
static bool OLED_I2C_Write(const uint8_t *data, uint16_t len){
    uint16_t sent = 0U;
    uint32_t timeout;

    if (data == NULL || len == 0U){
        return true;
    }

    /* 屏不在就别发：每条事务都要吃掉一次超时，几十条下来整机都卡住 */
    if (!OLED_PresenceCheck()){
        return false;
    }

    if (!OLED_I2C_WaitIdle()){
        return false;
    }

    DL_I2C_resetControllerTransfer(OLED_INST);

    while (sent < len && !DL_I2C_isControllerTXFIFOFull(OLED_INST)){
        DL_I2C_transmitControllerData(OLED_INST, data[sent++]);
    }

    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    DL_Common_delayCycles(3U);

    timeout = OLED_I2C_TIMEOUT;
    while (sent < len){
        uint32_t status = DL_I2C_getControllerStatus(OLED_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
            DL_I2C_resetControllerTransfer(OLED_INST);
            return false;
        }
        if (!DL_I2C_isControllerTXFIFOFull(OLED_INST)){
            DL_I2C_transmitControllerData(OLED_INST, data[sent++]);
            timeout = OLED_I2C_TIMEOUT;
        }
        else if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(OLED_INST);
            return false;
        }
    }

    timeout = OLED_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){
            DL_I2C_resetControllerTransfer(OLED_INST);
            return false;
        }
    }

    if ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
        DL_I2C_resetControllerTransfer(OLED_INST);
        return false;
    }
    return true;
}

/* 连续流式写入：支持单次连续写入最多 128 字节页显存，由 OLED_I2C_Write 硬件 FIFO 自动补充 */
static void OLED_Write_Bytes(uint8_t control, const uint8_t *data, uint16_t len){
    static uint8_t packet[130];
    if (data == NULL || len == 0U || len > 128U){
        return;
    }
    packet[0] = control;
    memcpy(&packet[1], data, len);
    (void)OLED_I2C_Write(packet, (uint16_t)(len + 1U));
}

//发送一个字节
//向SSD1306写入一个字节。
//mode:数据/命令标志 0,表示命令;1,表示数据;
static void OLED_WR_Byte(uint8_t dat,uint8_t mode){
    OLED_Write_Bytes((mode != 0U) ? 0x40U : 0x00U, &dat, 1U);
}
 
static void OLED_Write_ContinuousCmd(const uint8_t *cmds, uint8_t len){
    OLED_Write_Bytes(0x00U, cmds, len);
}
 
//坐标设置
static void OLED_Set_Pos(uint8_t x, uint8_t y){
//    OLED_WR_Byte(0xb0+y,OLED_CMD);
//    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
//    OLED_WR_Byte((x&0x0f),OLED_CMD);
    uint8_t cmds[3];
    cmds[0] = 0xb0 + y;                  // 页地址
    cmds[1] = ((x & 0xf0) >> 4) | 0x10;  // 列高地址
    cmds[2] = x & 0x0f;                  // 列低地址
    OLED_Write_ContinuousCmd(cmds, 3);   // 一次传输3个命令
}
 
//开启OLED显示    
void OLED_DisplayOn(void){
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
 
void OLED_Display_On(void){
    OLED_DisplayOn();
}

//关闭OLED显示     
void OLED_DisplayOff(void){
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}

void OLED_Display_Off(void){
    OLED_DisplayOff();
}

static void OLED_Write_ContinuousData(const uint8_t *data, uint16_t len){
    OLED_Write_Bytes(0x40U, data, len);
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void){  
//    uint8_t i,n;		    
//    for(i=0;i<8;i++)  
//    {  
//        OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
//        OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
//        OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
//        for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
//    } //更新显示
    static uint8_t clear_buf[128] = {0};  // 静态缓冲区，避免每次初始化
    // 循环8页，每页设置位置后连续写128字节0
    for (uint8_t i = 0; i < 8; i++){  
        OLED_Set_Pos(0, i);  // 优化后：一次I2C传输3个命令
        OLED_Write_ContinuousData(clear_buf, 128);
    }
}

void OLED_ClearLine(uint8_t y){
    static uint8_t clear_buf[128] = {0};

    if (y >= 8){
        return;
    }

    OLED_Set_Pos(0, y);
    OLED_Write_ContinuousData(clear_buf, 128);
}
 
//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63				 
//sizey:选择字体 6x8  8x16
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey){      	
    uint8_t c=0,sizex=sizey/2;
    uint16_t i=0,size1;

    if (sizey != 8 && sizey != 16){
        return;
    }

    if (y >= 8 || x >= 128){
        return;
    }

    if (chr < ' ' || chr > '~'){
        chr = '?';
    }

    if(sizey==8){
        size1=6;
    }
    else{
        size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
    }
    if ((uint16_t)x + sizex > 128){
        return;
    }

    c=chr-' ';//得到偏移后的值
    OLED_Set_Pos(x,y);
    for(i=0;i<size1;i++){
        if(i%sizex==0&&sizey!=8){
            OLED_Set_Pos(x,y++);
        }
        if(sizey==8) OLED_WR_Byte(asc2_0806[c][i],OLED_DATA);//6X8字号
        else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i],OLED_DATA);//8x16字号
        //		else if(sizey==xx) OLED_WR_Byte(asc2_xxxx[c][i],OLED_DATA);//用户添加字号
        else{
            return;
        }
    }
}
 
//m^n函数
static uint32_t oled_pow(uint8_t m,uint8_t n){
    uint32_t result=1;	 
    while(n--){
        result*=m;
    }
    return result;
}
 
//显示数字
//x,y :起点坐标
//num:要显示的数字
//len :数字的位数
//sizey:字体大小		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t sizey){         	
    uint8_t t,temp,m=0;
    uint8_t enshow=0;
    if(sizey==8){
        m=2;
    }
    for(t=0;t<len;t++){
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1)){
            if(temp==0){
                OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
                continue;
            }else enshow=1;
        }
        OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
    }
}
 
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,const char *chr,uint8_t sizey){
    uint8_t j=0;
    uint8_t step = (sizey == 8) ? 6 : (sizey / 2);

    if (chr == NULL || step == 0){
        return;
    }

    while (chr[j]!='\0' && x < 128){		
        if ((uint16_t)x + step > 128){
            break;
        }
        OLED_ShowChar(x,y,chr[j++],sizey);
        if(sizey==8){
            x+=6;
        }
        else{
            x+=sizey/2;
        }
    }
}

void OLED_ShowStringClearLine(uint8_t x,uint8_t y,const char *chr,uint8_t sizey){
    OLED_ClearLine(y);
    OLED_ShowString(x, y, chr, sizey);
}
 
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, const uint8_t *bmp){
    if (bmp == NULL || x >= OLED_WIDTH || y >= OLED_PAGE_COUNT || sizex == 0U || sizey == 0U){
        return;
    }

    uint8_t page_count = (uint8_t)((sizey + 7U) / 8U);
    uint16_t bmp_index = 0U;

    for (uint8_t page = 0U; page < page_count && (uint8_t)(y + page) < OLED_PAGE_COUNT; page++){
        OLED_Set_Pos(x, (uint8_t)(y + page));

        for (uint8_t col = 0U; col < sizex && (uint8_t)(x + col) < OLED_WIDTH; col++){
            OLED_WR_Byte(bmp[bmp_index + col], OLED_DATA);
        }

        bmp_index = (uint16_t)(bmp_index + sizex);
    }
}


//初始化SSD1306					    
void OLED_Init(void){
    OLED_DelayMs(100);
    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
    OLED_Clear();
    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
}

/* ================= 陀螺仪指南针罗盘仪表盘图形引擎 ================= */
static uint8_t s_oled_fb[8][128];

static void OLED_BufSetPixel(int x, int y, uint8_t color){
    if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
    if (color){
        s_oled_fb[y >> 3][x] |= (uint8_t)(1U << (y & 7));
    } else {
        s_oled_fb[y >> 3][x] &= (uint8_t)~(1U << (y & 7));
    }
}

static void OLED_BufDrawLine(int x0, int y0, int x1, int y1, uint8_t color){
    int dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 >= y0) ? (y1 - y0) : (y0 - y1);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (1){
        OLED_BufSetPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err << 1;
        if (e2 > -dy){
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx){
            err += dx;
            y0 += sy;
        }
    }
}

static void OLED_BufDrawCircle(int cx, int cy, int r, uint8_t color){
    int x = 0;
    int y = r;
    int d = 3 - (r << 1);

    while (x <= y){
        OLED_BufSetPixel(cx + x, cy + y, color);
        OLED_BufSetPixel(cx + y, cy + x, color);
        OLED_BufSetPixel(cx - x, cy + y, color);
        OLED_BufSetPixel(cx - y, cy + x, color);
        OLED_BufSetPixel(cx + x, cy - y, color);
        OLED_BufSetPixel(cx + y, cy - x, color);
        OLED_BufSetPixel(cx - x, cy - y, color);
        OLED_BufSetPixel(cx - y, cy - x, color);

        if (d < 0){
            d += (x << 2) + 6;
        } else {
            d += ((x - y) << 2) + 10;
            y--;
        }
        x++;
    }
}

static void OLED_BufDrawChar6x8(int x, int y, char c, uint8_t color){
    if (c < ' ' || c > '~') c = ' ';
    uint8_t idx = (uint8_t)(c - ' ');
    for (uint8_t col = 0; col < 6; col++){
        uint8_t bits = asc2_0806[idx][col];
        for (uint8_t b = 0; b < 8; b++){
            if (bits & (1U << b)){
                OLED_BufSetPixel(x + col, y + b, color);
            }
        }
    }
}

static void OLED_BufDrawString6x8(int x, int y, const char *str, uint8_t color){
    while (*str){
        OLED_BufDrawChar6x8(x, y, *str++, color);
        x += 6;
        if (x >= 128) break;
    }
}

static void OLED_BufDrawChar8x16(int x, int y, char c, uint8_t color){
    if (c < ' ' || c > '~') c = ' ';
    uint8_t idx = (uint8_t)(c - ' ');
    for (uint8_t col = 0; col < 8; col++){
        uint8_t bits_up   = asc2_1608[idx][col];
        uint8_t bits_down = asc2_1608[idx][col + 8];
        for (uint8_t b = 0; b < 8; b++){
            if (bits_up & (1U << b)){
                OLED_BufSetPixel(x + col, y + b, color);
            }
            if (bits_down & (1U << b)){
                OLED_BufSetPixel(x + col, y + 8 + b, color);
            }
        }
    }
}

static void OLED_BufDrawString8x16(int x, int y, const char *str, uint8_t color){
    while (*str){
        OLED_BufDrawChar8x16(x, y, *str++, color);
        x += 8;
        if (x >= 128) break;
    }
}

static int OLED_BufDrawCharArial(int x, int y, char c, const ArialGlyph_t *font_table, uint8_t color){
    if (c < ' ' || c > '~') c = ' ';
    uint8_t idx = (uint8_t)(c - ' ');
    const ArialGlyph_t *g = &font_table[idx];
    for (uint8_t col = 0; col < g->width; col++){
        uint16_t bits = g->cols[col];
        if (!bits) continue;
        int px = x + col;
        if (px >= 0 && px < 128){
            for (uint8_t b = 0; b < 16; b++){
                if (bits & (1U << b)){
                    int py = y + b;
                    if (py >= 0 && py < 64){
                        OLED_BufSetPixel(px, py, color);
                    }
                }
            }
        }
    }
    return g->advance;
}

static int OLED_BufGetStringWidthArial(const char *str, const ArialGlyph_t *font_table){
    int w = 0;
    while (*str){
        char c = *str++;
        if (c < ' ' || c > '~') c = ' ';
        w += font_table[(uint8_t)(c - ' ')].advance;
    }
    return w;
}

static int OLED_BufDrawStringArial(int x, int y, const char *str, const ArialGlyph_t *font_table, uint8_t color){
    int cur_x = x;
    while (*str){
        cur_x += OLED_BufDrawCharArial(cur_x, y, *str++, font_table, color);
    }
    return cur_x - x;
}

void OLED_DrawCompassDashboard(float yaw_deg,
                              uint8_t speed_pct,
                              const char *action,
                              float speed_m_s,
                              float dist_m,
                              uint8_t light){
    memset(s_oled_fb, 0, sizeof(s_oled_fb));

    /* 1. 左侧圆形指南针罗盘 (圆心: cx=28, cy=31, 半径: r=26) */
    int cx = 28;
    int cy = 31;
    int r = 26;
    OLED_BufDrawCircle(cx, cy, r, 1);

    /* 8 个方向刻度线 (0, 45, 90, 135, 180, 225, 270, 315 度) */
    static const int8_t s_sin_45[8] = { 0,  18,  25,  18,   0, -18, -25, -18};
    static const int8_t s_cos_45[8] = {25,  18,   0, -18, -25, -18,   0,  18};
    for (uint8_t i = 0; i < 8; i++){
        int x0 = cx + s_sin_45[i];
        int y0 = cy - s_cos_45[i];
        int x1 = cx + (s_sin_45[i] * 21) / 25;
        int y1 = cy - (s_cos_45[i] * 21) / 25;
        OLED_BufDrawLine(x0, y0, x1, y1, 1);
    }

    /* 四个主方位标字符 (N, S, W, E) */
    OLED_BufDrawChar6x8(cx - 3, cy - r + 4, 'N', 1);
    OLED_BufDrawChar6x8(cx - 3, cy + r - 11, 'S', 1);
    OLED_BufDrawChar6x8(cx - r + 4, cy - 4, 'W', 1);
    OLED_BufDrawChar6x8(cx + r - 9, cy - 4, 'E', 1);

    /* 2. 动态旋转指南针指针 (根据 yaw 角度动态指向) */
    float rad = yaw_deg * (3.14159265f / 180.0f);
    float s = sinf(rad);
    float c = cosf(rad);

    /* 北向箭头尖端 (长 19 像素) */
    int nx = cx + (int)(19.0f * s);
    int ny = cy - (int)(19.0f * c);

    /* 南向尾部 (长 14 像素) */
    int sx = cx - (int)(14.0f * s);
    int sy = cy + (int)(14.0f * c);

    /* 箭头双翼展开 (两侧各偏转约 145 度) */
    float rl = rad + 2.53f;
    float rr = rad - 2.53f;
    int lx = nx + (int)(7.0f * sinf(rl));
    int ly = ny - (int)(7.0f * cosf(rl));
    int rx = nx + (int)(7.0f * sinf(rr));
    int ry = ny - (int)(7.0f * cosf(rr));

    /* 绘制指南针指针主轴与三角形箭头 */
    OLED_BufDrawLine(cx, cy, nx, ny, 1);
    OLED_BufDrawLine(nx, ny, lx, ly, 1);
    OLED_BufDrawLine(nx, ny, rx, ry, 1);
    OLED_BufDrawLine(lx, ly, rx, ry, 1);
    OLED_BufDrawLine(cx, cy, sx, sy, 1);

    /* 中心轴心实心点 (3x3) */
    for (int dx = -1; dx <= 1; dx++){
        for (int dy = -1; dy <= 1; dy++){
            OLED_BufSetPixel(cx + dx, cy + dy, 1);
        }
    }

    (void)speed_pct;
    (void)speed_m_s;
    (void)dist_m;
    (void)light;

    /* 3. 右侧仪表区 (3 行纯净排版，已移除分割线，保留方位与指南针原始布局) */

    /* 行 1 (顶部 y=5): 专属品牌行 "MSP430 Car" (Arial Bold 10pt) */
    const char *title = "MSP430 Car";
    int w_title = OLED_BufGetStringWidthArial(title, g_arial_bold_10);
    int x_title = 58 + (70 - w_title) / 2;
    if (x_title < 59) x_title = 59;
    OLED_BufDrawStringArial(x_title, 5, title, g_arial_bold_10, 1);

    /* 计算当前方位文字 (N, NE, E, SE, S, SW, W, NW) */
    float norm_yaw = yaw_deg;
    while (norm_yaw < 0.0f) norm_yaw += 360.0f;
    while (norm_yaw >= 360.0f) norm_yaw -= 360.0f;
    const char *card = "N";
    if (norm_yaw >= 22.5f && norm_yaw < 67.5f)       card = "NE";
    else if (norm_yaw >= 67.5f && norm_yaw < 112.5f)  card = "E";
    else if (norm_yaw >= 112.5f && norm_yaw < 157.5f) card = "SE";
    else if (norm_yaw >= 157.5f && norm_yaw < 202.5f) card = "S";
    else if (norm_yaw >= 202.5f && norm_yaw < 247.5f) card = "SW";
    else if (norm_yaw >= 247.5f && norm_yaw < 292.5f) card = "W";
    else if (norm_yaw >= 292.5f && norm_yaw < 337.5f) card = "NW";

    char buf[16];

    /* 行 2 (中部 y=24): 当前航向方位 "DIR: %s" (Arial Bold 13pt) */
    snprintf(buf, sizeof(buf), "DIR: %s", card);
    int w_dir = OLED_BufGetStringWidthArial(buf, g_arial_bold_13);
    int x_dir = 58 + (70 - w_dir) / 2;
    if (x_dir < 59) x_dir = 59;
    OLED_BufDrawStringArial(x_dir, 24, buf, g_arial_bold_13, 1);

    /* 行 3 (底部 y=46): 当前工况动作 "ACT:%s" (Arial Bold 13pt) */
    const char *act_raw = (action != NULL) ? action : "STOP";
    char act_disp[8];
    if (strcmp(act_raw, "TRACK") == 0) {
        strncpy(act_disp, "TRK", sizeof(act_disp));
    } else if (strcmp(act_raw, "TIMEOUT") == 0) {
        strncpy(act_disp, "TIME", sizeof(act_disp));
    } else if (strcmp(act_raw, "RESET") == 0) {
        strncpy(act_disp, "RST", sizeof(act_disp));
    } else {
        strncpy(act_disp, act_raw, sizeof(act_disp));
        act_disp[5] = '\0';
    }
    snprintf(buf, sizeof(buf), "ACT:%s", act_disp);
    int w_act = OLED_BufGetStringWidthArial(buf, g_arial_bold_13);
    int x_act = 58 + (70 - w_act) / 2;
    if (x_act < 59) x_act = 59;
    OLED_BufDrawStringArial(x_act, 46, buf, g_arial_bold_13, 1);

    /* 5. 一次性将显存刷新至 SSD1306 屏幕 */
    for (uint8_t page = 0; page < 8; page++){
        OLED_Set_Pos(0, page);
        OLED_Write_ContinuousData(s_oled_fb[page], 128);
    }
} 

void OLED_DrawGrayscaleMonitor(const uint8_t analog[8], uint8_t digital_mask, bool online)
{
    memset(s_oled_fb, 0, sizeof(s_oled_fb));

    if (!online || analog == NULL){
        /* 离线时别画一排 0 让人误以为"全是白"，直接把结论写在屏上 */
        OLED_BufDrawString8x16(16, 8, "NO SENSOR", 1);
        OLED_BufDrawString6x8(20, 36, "CHECK I2C 0x4C", 1);
        for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++){
            OLED_Set_Pos(0, page);
            OLED_Write_ContinuousData(s_oled_fb[page], OLED_WIDTH);
        }
        return;
    }

    /* 2 列 × 4 格，每格 62×16px：
     *   上 8px：通道号 + 黑白方块 + 三位数值
     *   下 8px：模拟量条形图（与上面同一列宽，一眼对得上号）
     * 单列塞 8 行时字号只有 6x8、行行相接，实测难读；拆两列后每格
     * 留出 8px 的空白行，读数、看条都不费劲。 */
    for (uint8_t i = 0U; i < 8U; i++){
        int col = (i < 4U) ? 0 : 1;
        int row = (int)(i % 4U);
        int x0 = (col == 0) ? 0 : 66;
        int y0 = row * 16;
        uint8_t v = analog[i];

        /* 通道号 1~8 */
        OLED_BufDrawChar6x8(x0, y0, (char)('1' + i), 1);

        /* 数字量指示：实心=压黑，空心=白底 */
        int dx = x0 + 8;
        int dy = y0 + 2;
        if ((digital_mask & (uint8_t)(1U << i)) != 0U){
            for (int yy = 0; yy < 4; yy++){
                for (int xx = 0; xx < 4; xx++){
                    OLED_BufSetPixel(dx + xx, dy + yy, 1);
                }
            }
        } else {
            for (int t = 0; t < 4; t++){
                OLED_BufSetPixel(dx + t, dy, 1);
                OLED_BufSetPixel(dx + t, dy + 3, 1);
                OLED_BufSetPixel(dx, dy + t, 1);
                OLED_BufSetPixel(dx + 3, dy + t, 1);
            }
        }

        /* 三位数值右对齐（百位/十位为 0 时留空，不画前导零） */
        char num[3];
        num[0] = (v >= 100U) ? (char)('0' + v / 100U) : ' ';
        num[1] = (v >= 10U)  ? (char)('0' + (v / 10U) % 10U) : ' ';
        num[2] = (char)('0' + v % 10U);
        for (uint8_t k = 0U; k < 3U; k++){
            OLED_BufDrawChar6x8(x0 + 41 + (int)k * 6, y0, num[k], 1);
        }

        /* 条形图：整格宽 60px，6px 高，紧贴数值下方 */
        int w = ((int)v * 60) / 255;
        if (v > 0U && w == 0){
            w = 1;
        }
        for (int yy = 0; yy < 6; yy++){
            for (int xx = 0; xx < w; xx++){
                OLED_BufSetPixel(x0 + xx, y0 + 9 + yy, 1);
            }
        }
    }

    /* 左右两列的分隔线，避免两格的数值看串行 */
    for (int y = 0; y < (int)OLED_HEIGHT; y++){
        OLED_BufSetPixel(62, y, 1);
    }

    for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++){
        OLED_Set_Pos(0, page);
        OLED_Write_ContinuousData(s_oled_fb[page], OLED_WIDTH);
    }
}

/* ═══════════════ 校准向导整屏汉字提示 ═══════════════
 * 旧版把英文标题、两行指令、6x8 小字状态全塞进 128x64，字号小、信息重复，
 * 站远一点根本看不清。现在只留两行：
 *   第 1 行 24px 汉字标题（循线校准 / 校准成功 / 校准失败）
 *   第 2 行 16px 汉字提示（请放在白色上 / 请放在黑色上 / 请重试）
 * 两行水平居中，标题上留 8px、行间 8px，视觉重心正好在屏幕中线。
 */

#define OLED_CN24_ADV  26   /* 24px 字身 24 + 2px 字距 */
#define OLED_CN16_ADV  17   /* 16px 字身 16 + 1px 字距 */

static void OLED_BufDrawCN24(int x, int y, uint8_t idx)
{
    if (idx >= (uint8_t)(sizeof(g_oled_font_cn24) / sizeof(g_oled_font_cn24[0]))){
        return;
    }
    for (int row = 0; row < 24; row++){
        uint32_t bits = g_oled_font_cn24[idx][row];
        if (bits == 0U){
            continue;
        }
        for (int col = 0; col < 24; col++){
            if ((bits & (1UL << col)) != 0U){
                OLED_BufSetPixel(x + col, y + row, 1U);
            }
        }
    }
}

static void OLED_BufDrawCN16(int x, int y, uint8_t idx)
{
    if (idx >= (uint8_t)(sizeof(g_oled_font_cn16) / sizeof(g_oled_font_cn16[0]))){
        return;
    }
    for (int row = 0; row < 16; row++){
        uint16_t bits = g_oled_font_cn16[idx][row];
        if (bits == 0U){
            continue;
        }
        for (int col = 0; col < 16; col++){
            if ((bits & (1U << col)) != 0U){
                OLED_BufSetPixel(x + col, y + row, 1U);
            }
        }
    }
}

static void OLED_BufDrawCNLine24(const uint8_t *ids, uint8_t n, int y)
{
    int x = (int)(OLED_WIDTH - ((int)n * OLED_CN24_ADV - 2)) / 2;
    for (uint8_t i = 0U; i < n; i++){
        OLED_BufDrawCN24(x, y, ids[i]);
        x += OLED_CN24_ADV;
    }
}

static void OLED_BufDrawCNLine16(const uint8_t *ids, uint8_t n, int y)
{
    int x = (int)(OLED_WIDTH - ((int)n * OLED_CN16_ADV - 1)) / 2;
    for (uint8_t i = 0U; i < n; i++){
        OLED_BufDrawCN16(x, y, ids[i]);
        x += OLED_CN16_ADV;
    }
}

void OLED_DrawCalibrationScreen(uint8_t step)
{
    /* 循线校准 */
    static const uint8_t s_title_cal[] = {CN24_IDX_XUN, CN24_IDX_XIAN,
                                          CN24_IDX_JIAO, CN24_IDX_ZHUN};
    /* 校准成功 */
    static const uint8_t s_title_ok[]  = {CN24_IDX_JIAO, CN24_IDX_ZHUN,
                                          CN24_IDX_CHENG, CN24_IDX_GONG};
    /* 校准失败 */
    static const uint8_t s_title_err[] = {CN24_IDX_JIAO, CN24_IDX_ZHUN,
                                          CN24_IDX_SHI, CN24_IDX_BAI};
    /* 请放在白色上 */
    static const uint8_t s_body_white[] = {CN16_IDX_QING, CN16_IDX_FANG, CN16_IDX_ZAI,
                                           CN16_IDX_BAI, CN16_IDX_SE, CN16_IDX_SHANG};
    /* 请放在黑色上 */
    static const uint8_t s_body_black[] = {CN16_IDX_QING, CN16_IDX_FANG, CN16_IDX_ZAI,
                                           CN16_IDX_HEI, CN16_IDX_SE, CN16_IDX_SHANG};
    /* 请重试 */
    static const uint8_t s_body_retry[] = {CN16_IDX_QING, CN16_IDX_ZHONG, CN16_IDX_SHI3};

    memset(s_oled_fb, 0, sizeof(s_oled_fb));

    if (step == OLED_CALIB_STEP_OK){
        /* 成功只有一句，单独居中，别让空行把画面劈成两半 */
        OLED_BufDrawCNLine24(s_title_ok, 4U, 20);
    } else if (step == OLED_CALIB_STEP_ERR){
        OLED_BufDrawCNLine24(s_title_err, 4U, 8);
        OLED_BufDrawCNLine16(s_body_retry, 3U, 40);
    } else {
        OLED_BufDrawCNLine24(s_title_cal, 4U, 8);
        if (step == OLED_CALIB_STEP_BLACK){
            OLED_BufDrawCNLine16(s_body_black, 6U, 40);
        } else {
            OLED_BufDrawCNLine16(s_body_white, 6U, 40);
        }
    }

    for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++){
        OLED_Set_Pos(0, page);
        OLED_Write_ContinuousData(s_oled_fb[page], OLED_WIDTH);
    }
}
