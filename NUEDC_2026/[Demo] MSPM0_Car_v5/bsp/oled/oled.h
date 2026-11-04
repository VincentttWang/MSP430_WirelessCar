/**
 * @file  oled.h
 * @brief SSD1306 OLED 显示接口。
 */

#ifndef OLED_H
#define OLED_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 校准向导的四种整屏提示（128x64；一行 24px 汉字标题 + 一行 16px 提示） */
#define OLED_CALIB_STEP_WHITE  1U   /* 循线校准 / 请放在白色上 */
#define OLED_CALIB_STEP_BLACK  2U   /* 循线校准 / 请放在黑色上 */
#define OLED_CALIB_STEP_OK     3U   /* 校准成功 */
#define OLED_CALIB_STEP_ERR    4U   /* 校准失败 / 请重试 */

/**
 * @brief 绘制传感器校准引导界面（汉字整屏提示）
 * @param step OLED_CALIB_STEP_xxx
 */
void OLED_DrawCalibrationScreen(uint8_t step);

/**
 * @brief 绘制"八路灰度实时监视"整屏（UP 键切进来）
 * @param analog      8 路模拟量，0~255
 * @param digital_mask 数字量位图，bit0 = 探头 1，1 = 检测到黑
 * @param online      传感器是否在线（离线时整屏提示 NO SENSOR）
 */
void OLED_DrawGrayscaleMonitor(const uint8_t analog[8], uint8_t digital_mask, bool online);

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
#define OLED_PAGE_COUNT 8U

/**
 * @brief 初始化 SSD1306 OLED。
 */
void OLED_Init(void);

/**
 * @brief 打开 OLED 显示。
 */
void OLED_DisplayOn(void);

/**
 * @brief 关闭 OLED 显示。
 */
void OLED_DisplayOff(void);

/**
 * @brief 兼容旧接口的打开显示函数。
 */
void OLED_Display_On(void);

/**
 * @brief 兼容旧接口的关闭显示函数。
 */
void OLED_Display_Off(void);

/**
 * @brief 设置正常/反色显示。
 * @param enable 非 0 表示反色显示，0 表示正常显示。
 */
void OLED_ColorTurn(uint8_t enable);

/**
 * @brief 清空整屏。
 */
void OLED_Clear(void);

/**
 * @brief 清空指定页。
 */
void OLED_ClearLine(uint8_t page);

/**
 * @brief 显示单个 ASCII 字符。
 */
void OLED_ShowChar(uint8_t x, uint8_t page, uint8_t chr, uint8_t sizey);

/**
 * @brief 显示无符号整数。
 */
void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len, uint8_t sizey);

/**
 * @brief 显示字符串。
 */
void OLED_ShowString(uint8_t x, uint8_t page, const char *chr, uint8_t sizey);

/**
 * @brief 清空指定页后显示字符串。
 */
void OLED_ShowStringClearLine(uint8_t x, uint8_t page, const char *chr, uint8_t sizey);

/**
 * @brief 绘制位图。
 * @param x 起始列。
 * @param page 起始页。
 * @param sizex 位图宽度，单位 px。
 * @param sizey 位图高度，单位 px。
 * @param bmp 位图数据。
 */
void OLED_DrawBMP(uint8_t x, uint8_t page, uint8_t sizex, uint8_t sizey, const uint8_t *bmp);

/**
 * @brief 绘制陀螺仪航向“指南针”式罗盘仪表盘全景界面。
 * @param yaw_deg 当前偏航角 (-180.0 ~ +180.0 或 0 ~ 360.0)。
 * @param speed_pct 当前限速百分比 (0 ~ 100)。
 * @param action 当前运动动作名称 ("STOP", "FWD", "REV", "LEFT", "RIGHT")。
 * @param speed_m_s 当前对地线速度 (m/s)。
 * @param dist_m 当前累计行驶里程 (m)。
 * @param light 交通信号灯状态 (0=RED, 1=GREEN)。
 */
void OLED_DrawCompassDashboard(float yaw_deg,
                              uint8_t speed_pct,
                              const char *action,
                              float speed_m_s,
                              float dist_m,
                              uint8_t light);

#ifdef __cplusplus
}
#endif

#endif /* OLED_H */
