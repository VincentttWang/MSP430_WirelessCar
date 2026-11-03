/**
 * @file  grayscale_sensor.h
 * @brief BSP 8 路灰度传感器接口（支持感为 I2C 八路灰度与 GPIO 兼容模式）。
 */
#ifndef GRAYSCALE_SENSOR_H
#define GRAYSCALE_SENSOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRAYSCALE_SENSOR_CHANNEL_COUNT 8U

/* ═══════════════ 感为 I2C 八路灰度模块定义 ═══════════════
 * 模块硬件连接：MSPM0G3507 I2C0 (PA0=SDA, PA1=SCL)
 * 7位 I2C 地址: 0x4C (默认)
 * 寄存器:
 *   0xAA: Ping 握手，正常应答返回 0x66
 *   0xDD: 8路数字量读取，1字节，bit0~7分别对应探头1~8 (黑线=1, 白底=0)
 */
#define GW_GRAY_I2C_ADDR         0x4CU
#define GW_GRAY_REG_DIGITAL      0xDDU
#define GW_GRAY_REG_PING         0xAAU
#define GW_GRAY_PING_RESPONSE    0x66U
#define GW_GRAY_REG_ANALOG_BASE  0xB0U
#define GW_GRAY_REG_CALIB_BLACK  0xD0U
#define GW_GRAY_REG_CALIB_WHITE  0xD1U
#define GW_GRAY_REG_NORMALIZE    0xCFU

#ifndef GRAYSCALE_SENSOR_1_PORT
#define GRAYSCALE_SENSOR_1_PORT Tracking_Tracking_1_PORT
#endif
#ifndef GRAYSCALE_SENSOR_1_PIN
#define GRAYSCALE_SENSOR_1_PIN Tracking_Tracking_1_PIN
#endif
#ifndef GRAYSCALE_SENSOR_2_PORT
#define GRAYSCALE_SENSOR_2_PORT Tracking_Tracking_2_PORT
#endif
#ifndef GRAYSCALE_SENSOR_2_PIN
#define GRAYSCALE_SENSOR_2_PIN Tracking_Tracking_2_PIN
#endif
#ifndef GRAYSCALE_SENSOR_3_PORT
#define GRAYSCALE_SENSOR_3_PORT Tracking_Tracking_3_PORT
#endif
#ifndef GRAYSCALE_SENSOR_3_PIN
#define GRAYSCALE_SENSOR_3_PIN Tracking_Tracking_3_PIN
#endif
#ifndef GRAYSCALE_SENSOR_4_PORT
#define GRAYSCALE_SENSOR_4_PORT Tracking_Tracking_4_PORT
#endif
#ifndef GRAYSCALE_SENSOR_4_PIN
#define GRAYSCALE_SENSOR_4_PIN Tracking_Tracking_4_PIN
#endif
#ifndef GRAYSCALE_SENSOR_5_PORT
#define GRAYSCALE_SENSOR_5_PORT Tracking_Tracking_5_PORT
#endif
#ifndef GRAYSCALE_SENSOR_5_PIN
#define GRAYSCALE_SENSOR_5_PIN Tracking_Tracking_5_PIN
#endif
#ifndef GRAYSCALE_SENSOR_6_PORT
#define GRAYSCALE_SENSOR_6_PORT Tracking_Tracking_6_PORT
#endif
#ifndef GRAYSCALE_SENSOR_6_PIN
#define GRAYSCALE_SENSOR_6_PIN Tracking_Tracking_6_PIN
#endif
#ifndef GRAYSCALE_SENSOR_7_PORT
#define GRAYSCALE_SENSOR_7_PORT Tracking_Tracking_7_PORT
#endif
#ifndef GRAYSCALE_SENSOR_7_PIN
#define GRAYSCALE_SENSOR_7_PIN Tracking_Tracking_7_PIN
#endif
#ifndef GRAYSCALE_SENSOR_8_PORT
#define GRAYSCALE_SENSOR_8_PORT Tracking_Tracking_8_PORT
#endif
#ifndef GRAYSCALE_SENSOR_8_PIN
#define GRAYSCALE_SENSOR_8_PIN Tracking_Tracking_8_PIN
#endif

#ifndef GRAYSCALE_SENSOR_ACTIVE_LOW
#define GRAYSCALE_SENSOR_ACTIVE_LOW 1U
#endif

typedef enum {
    GRAYSCALE_SENSOR_CHANNEL_0 = 0,
    GRAYSCALE_SENSOR_CHANNEL_1,
    GRAYSCALE_SENSOR_CHANNEL_2,
    GRAYSCALE_SENSOR_CHANNEL_3,
    GRAYSCALE_SENSOR_CHANNEL_4,
    GRAYSCALE_SENSOR_CHANNEL_5,
    GRAYSCALE_SENSOR_CHANNEL_6,
    GRAYSCALE_SENSOR_CHANNEL_7,
    GRAYSCALE_SENSOR_CHANNEL_MAX
} GRAYSCALE_SENSOR_CHANNEL;

/**
 * @brief 初始化灰度传感器（检测感为 I2C 模块是否在线）。
 */
void GrayscaleSensor_Init(void);

/**
 * @brief 动态探测感为 I2C 灰度从机是否在线。
 * @return true 探测成功且在线，false 无应答/离线
 */
bool GrayscaleSensor_Probe(void);

/**
 * @brief 查询感为 I2C 灰度传感器是否在线可用。
 * @return true 在线，false 未接或通信异常
 */
bool GrayscaleSensor_IsI2cOnline(void);
uint8_t GrayscaleSensor_GetI2cAddr(void);

/**
 * @brief 读取全部 8 路灰度传感器数字状态。
 * @param digital_array 输出数组，长度必须不小于 GRAYSCALE_SENSOR_CHANNEL_COUNT。
 * @note 遵循巡线核心统一语义：0 表示检测到黑线，1 表示白底。
 */
void GrayscaleSensor_Read(uint8_t digital_array[GRAYSCALE_SENSOR_CHANNEL_COUNT]);

/**
 * @brief 读取 8 路灰度传感器位掩码。
 * @return bit0~bit7 对应通道 0~7，置 1 表示检测到黑线。
 */
uint8_t GrayscaleSensor_ReadMask(void);

/**
 * @brief 读取单个灰度传感器通道。
 * @return 0 表示检测到黑线，1 表示白底。
 */
uint8_t GrayscaleSensor_ReadSingle(GRAYSCALE_SENSOR_CHANNEL channel);

#define TrackingSensor_Read(digital_array) GrayscaleSensor_Read(digital_array)

/**
 * @brief 读取 8 路灰度传感器模拟量（0~255）。
 * @param analog_array 长度至少为 8 的输出数组。
 * @return true 成功读取，false 失败
 */
bool GrayscaleSensor_ReadAnalog(uint8_t analog_array[GRAYSCALE_SENSOR_CHANNEL_COUNT]);

/**
 * @brief 采集白底基准数据并存入内部校准缓冲区。
 * @param out_white 可选输出数组（可传 NULL），输出 8 路采集均值。
 * @return true 成功，false 失败
 */
bool GrayscaleSensor_CalibrateSampleWhite(uint8_t out_white[GRAYSCALE_SENSOR_CHANNEL_COUNT]);

/**
 * @brief 采集黑线基准数据并存入内部校准缓冲区。
 * @param out_black 可选输出数组（可传 NULL），输出 8 路采集均值。
 * @return true 成功，false 失败
 */
bool GrayscaleSensor_CalibrateSampleBlack(uint8_t out_black[GRAYSCALE_SENSOR_CHANNEL_COUNT]);

/**
 * @brief 根据已采样的白底和黑线数据，按感为官方例程算法计算滞回比较阈值并写入传感器寄存器。
 * @param out_th_w 可选输出白阈值数组（可传 NULL）
 * @param out_th_b 可选输出黑阈值数组（可传 NULL）
 * @return true 校准并写入成功，false 失败
 */
bool GrayscaleSensor_ApplyCalibration(uint8_t out_th_w[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                      uint8_t out_th_b[GRAYSCALE_SENSOR_CHANNEL_COUNT]);

/**
 * @brief 获取最近一次校准所记录的白底、黑线与阈值数据。
 */
void GrayscaleSensor_GetCalibrationData(uint8_t white[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                       uint8_t black[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                       uint8_t th_white[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                       uint8_t th_black[GRAYSCALE_SENSOR_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* GRAYSCALE_SENSOR_H */
