/**
 * @file  app_remote_car_task.h
 * @brief MSPM0G3507 智能小车双模遥控与 MSP430 协同任务
 */
#ifndef APP_REMOTE_CAR_TASK_H
#define APP_REMOTE_CAR_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 运行小车无线遥控与双机协同任务
 */
void App_RunRemoteCarTask(void);

/**
 * @brief 处理单字节远程控制指令 (视觉 K230 通道使用，保持原语义)
 */
void AppRemoteCar_ProcessRxByte(uint8_t byte);

/**
 * @brief 处理来自上位机无线透传模块(UART1 / PA8-PA9)的字节
 */
void AppRemoteCar_ProcessPcByte(uint8_t byte);

/**
 * @brief 处理来自 MSP430 显示终端(UART0 / PB0-PB1)的字节
 */
void AppRemoteCar_ProcessMsp430Byte(uint8_t byte);

/**
 * @brief MSP430 终端是否还活着（timeout_ms 内收到过任何字节，含 2Hz 心跳）
 *
 * 天猛星上的黄色 LED 直接反映这个状态：
 *   黄灯亮 = MSP430 在跑且在正确发数据
 *   黄灯灭 = MSP430 没运行 / 没接线 / 波特率不对
 * 这样即使 MSP430 自己黑屏，也能一眼判断它到底有没有在工作。
 */
bool AppRemoteCar_IsMsp430Alive(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_REMOTE_CAR_TASK_H */
