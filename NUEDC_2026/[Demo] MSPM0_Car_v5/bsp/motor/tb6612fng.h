/**
 * @file  tb6612fng.h
 * @brief BSP TB6612FNG 双路直流电机驱动接口。
 */
#ifndef TB6612FNG_H
#define TB6612FNG_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TB6612FNG_LEFT_IN1_PORT
#define TB6612FNG_LEFT_IN1_PORT Motor_IO_BIN1_PORT
#endif

#ifndef TB6612FNG_LEFT_IN1_PIN
#define TB6612FNG_LEFT_IN1_PIN Motor_IO_BIN1_PIN
#endif

#ifndef TB6612FNG_LEFT_IN2_PORT
#define TB6612FNG_LEFT_IN2_PORT Motor_IO_BIN2_PORT
#endif

#ifndef TB6612FNG_LEFT_IN2_PIN
#define TB6612FNG_LEFT_IN2_PIN Motor_IO_BIN2_PIN
#endif

#ifndef TB6612FNG_LEFT_PWM_TIMER
#define TB6612FNG_LEFT_PWM_TIMER Motor_INST
#endif

/* 左轮 = TB6612 的 B 通道，PWMB 接 PA7 = TIMG7_CCP1（原理图 P1-7） */
#ifndef TB6612FNG_LEFT_PWM_CHANNEL
#define TB6612FNG_LEFT_PWM_CHANNEL DL_TIMER_CC_1_INDEX
#endif

/* 右轮 = TB6612 的 A 通道。
 * AIN1 = PB17（P1-2），AIN2 = PA16（P1-3），PWMA = PB10（P1-1）。
 * 注意 ti_msp_dl_config.h 里 Motor_IO_AIN1_PIN 这个"名字"其实指的是
 * 网络名 AIN1，而它接的是 MCU 的 PA16 —— 为避免歧义这里直接写端口和引脚。 */
#ifndef TB6612FNG_RIGHT_IN1_PORT
#define TB6612FNG_RIGHT_IN1_PORT GPIOB
#endif

#ifndef TB6612FNG_RIGHT_IN1_PIN
#define TB6612FNG_RIGHT_IN1_PIN DL_GPIO_PIN_17
#endif

#ifndef TB6612FNG_RIGHT_IN2_PORT
#define TB6612FNG_RIGHT_IN2_PORT GPIOA
#endif

#ifndef TB6612FNG_RIGHT_IN2_PIN
#define TB6612FNG_RIGHT_IN2_PIN DL_GPIO_PIN_16
#endif

/* PWMA 在 PB10，PB10 只能复用 TIMG0_CCP0（无 TIMG7 复用） */
#ifndef TB6612FNG_RIGHT_PWM_TIMER
#define TB6612FNG_RIGHT_PWM_TIMER MotorA_INST
#endif

#ifndef TB6612FNG_RIGHT_PWM_CHANNEL
#define TB6612FNG_RIGHT_PWM_CHANNEL DL_TIMER_CC_0_INDEX
#endif

#ifndef TB6612FNG_PWM_PERIOD
#define TB6612FNG_PWM_PERIOD 1000U
#endif

/* ── 方向修正 ─────────────────────────────────────────────────────────
 * 约定：**占空比为正是"车头朝前"**，左右两个通道必须一致。
 *
 * 下面两个宏故意取值不同，不是笔误：
 *   · 右通道在上面就把 AIN1 / AIN2 两个网络名的位置换了过来
 *     （RIGHT_IN1 实际接的是网络 AIN2 = PB17），这次"换位"本身
 *     等价于一次取反；
 *   · 左通道是直连（BIN1 = PB18 / BIN2 = PB19），没有这层换位。
 * 所以要让两边都满足"正 = 前进"：右通道需要 INVERT=1 把换位抵掉，
 * 左通道必须保持 INVERT=0。两个都写 1 就等于把左轮单独反了一次。
 *
 * 历史现象（两个都写 1 时，实测）：
 *   W(+,+)  原地顺时针转（右轮前进、左轮后退）—— 看起来像"前进变右转"
 *   A(-,+)  直行前进（两边都前进）            —— 看起来像"左转变直行"
 *   D(+,-)  直行后退（两边都后退）            —— 看起来像"右转变后退"
 *
 * 以后若换了电机插头方向，只改对应的那一个宏：
 *   前进正常但转向反了 -> 两个宏同时取反；
 *   前进变成原地打转   -> 只有一个宏取反。
 */
#ifndef TB6612FNG_LEFT_INVERT
#define TB6612FNG_LEFT_INVERT 0
#endif

#ifndef TB6612FNG_RIGHT_INVERT
#define TB6612FNG_RIGHT_INVERT 1
#endif

/**
 * @brief TB6612FNG 电机通道。
 */
typedef enum {
    TB6612FNG_CHANNEL_LEFT = 0,
    TB6612FNG_CHANNEL_RIGHT,
    TB6612FNG_CHANNEL_MAX
} TB6612FNG_CHANNEL;

/**
 * @brief TB6612FNG 输出状态。
 */
typedef enum {
    /** 断开输出，电机滑行。 */
    TB6612FNG_OUTPUT_COAST = 0,
    /** 正向输出。 */
    TB6612FNG_OUTPUT_FORWARD,
    /** 反向输出。 */
    TB6612FNG_OUTPUT_BACKWARD,
    /** 主动刹车。 */
    TB6612FNG_OUTPUT_BRAKE
} TB6612FNG_OUTPUT;

/**
 * @brief 初始化 TB6612FNG 双路电机输出。
 */
BSP_STATUS TB6612FNG_Init(void);

/**
 * @brief 设置指定通道占空比。
 * @param channel 电机通道。
 * @param duty_percent 占空比百分比，正负表示方向，范围会限制到 [-100, 100]。
 */
BSP_STATUS TB6612FNG_SetDuty(TB6612FNG_CHANNEL channel, float duty_percent);

/**
 * @brief 主动刹车指定通道。
 */
BSP_STATUS TB6612FNG_Brake(TB6612FNG_CHANNEL channel);

/**
 * @brief 关闭指定通道输出，使电机滑行。
 */
BSP_STATUS TB6612FNG_Coast(TB6612FNG_CHANNEL channel);

/**
 * @brief 主动刹车全部通道。
 */
BSP_STATUS TB6612FNG_BrakeAll(void);

/**
 * @brief 关闭全部通道输出。
 */
BSP_STATUS TB6612FNG_CoastAll(void);

/**
 * @brief 获取指定通道最近设置的占空比。
 */
float TB6612FNG_GetDuty(TB6612FNG_CHANNEL channel);

/**
 * @brief 获取指定通道最近输出状态。
 */
TB6612FNG_OUTPUT TB6612FNG_GetOutputStatus(TB6612FNG_CHANNEL channel);

#ifdef __cplusplus
}
#endif

#endif /* TB6612FNG_H */
