/**
 * @file  hall_encoder.h
 * @brief BSP 霍尔编码器接口。
 */
#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include "bsp_common.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HALL_ENCODER_A_PORT
#define HALL_ENCODER_A_PORT Motor_IO_E1A_PORT
#endif

#ifndef HALL_ENCODER_A_PIN
#define HALL_ENCODER_A_PIN Motor_IO_E1A_PIN
#endif

#ifndef HALL_ENCODER_B_PORT
#define HALL_ENCODER_B_PORT Motor_IO_E1A_PORT
#endif

#ifndef HALL_ENCODER_B_PIN
#define HALL_ENCODER_B_PIN Motor_IO_E1B_PIN
#endif

#ifndef HALL_ENCODER_GPIO_IRQN
#define HALL_ENCODER_GPIO_IRQN GPIOA_INT_IRQn
#endif

#ifndef HALL_ENCODER_SAMPLE_TIMER
#define HALL_ENCODER_SAMPLE_TIMER TIMER_0_INST
#endif

#ifndef HALL_ENCODER_SAMPLE_TIMER_IRQN
#define HALL_ENCODER_SAMPLE_TIMER_IRQN TIMER_0_INST_INT_IRQN
#endif

#ifndef HALL_ENCODER_PI
#define HALL_ENCODER_PI 3.1415926f
#endif

#ifndef HALL_ENCODER_PPR
#define HALL_ENCODER_PPR 13.0f
#endif

/* 每个正交周期实际进中断的次数。
 * GROUP1_IRQHandler 同时打开了 A、B 两路的**上升沿**中断，
 * 一个电周期里正好有 2 个上升沿（A 一次、B 一次），所以这里是 2。
 * 旧代码按 1 次算，但 ISR 实际记了 2 次 —— 仅这一项就让所有速度/里程
 * 翻了一倍。若以后把极性改成 RISE|FALL（4 倍频），这里必须同步改成 4。 */
#ifndef HALL_ENCODER_COUNTS_PER_CYCLE
#define HALL_ENCODER_COUNTS_PER_CYCLE 2.0f
#endif

#ifndef HALL_ENCODER_REDUCTION_RATIO
#define HALL_ENCODER_REDUCTION_RATIO 28.0f
#endif

#ifndef HALL_ENCODER_WHEEL_DIAMETER_M
#define HALL_ENCODER_WHEEL_DIAMETER_M 0.065f
#endif

/* 必须与 TIMER_0(TIMA0) 的实际溢出周期一致：
 *   TIMER_0_INST_LOAD_VALUE = 3999，时钟 BUSCLK/8/(0+1) = 40kHz
 *   -> (3999+1)/40000 = 0.1 s
 * 旧值写的是 0.01f，比真实采样周期小 10 倍，速度又被放大 10 倍。
 * 二者叠加（2 x 10）就是"测速偏快很多"的全部原因。 */
#ifndef HALL_ENCODER_SAMPLE_PERIOD_S
#define HALL_ENCODER_SAMPLE_PERIOD_S 0.1f
#endif

/* 轮胎打滑/直径公差的经验修正。没有实测标定就保持 1.00，
 * 否则会凭空多出 5% 的系统误差。 */
#ifndef HALL_ENCODER_DISTANCE_SCALE
#define HALL_ENCODER_DISTANCE_SCALE 1.0f
#endif

/* 编码器 A/B 相序决定"正转"时计数是递增还是递减。
 * 电机方向在 tb6612fng.h 里做软件取反后，编码器符号也跟着反了一次，
 * 所以这里留一个开关：
 *   车向前开、屏幕 V 显示为负数  -> 改成 1
 *   车向前开、屏幕 V 显示为正数  -> 保持 0
 * 只影响符号，不影响速度/里程的绝对值。 */
#ifndef HALL_ENCODER_DIR_INVERT
#define HALL_ENCODER_DIR_INVERT 0
#endif

/**
 * @brief 霍尔编码器方向。
 */
typedef enum {
    HALL_ENCODER_DIR_FORWARD = 0,
    HALL_ENCODER_DIR_REVERSE
} HALL_ENCODER_DIR;

/**
 * @brief 初始化霍尔编码器计数和采样状态。
 */
BSP_STATUS HallEncoder_Init(void);

/**
 * @brief 处理编码器 GPIO 中断。
 * @param gpio_status GPIO 中断状态寄存器值。
 */
void HallEncoder_HandleGpioIrq(uint32_t gpio_status);

/**
 * @brief 更新一次速度采样。
 * @note 通常由固定周期定时器中断调用。
 */
void HallEncoder_UpdateSample(void);

/**
 * @brief 获取累计编码器计数。
 */
int32_t HallEncoder_GetCount(void);

/**
 * @brief 获取最近判断的编码器方向。
 */
HALL_ENCODER_DIR HallEncoder_GetDir(void);

/**
 * @brief 获取估算线速度，单位 m/s。
 */
float HallEncoder_GetSpeed(void);

/**
 * @brief 获取累计距离估计，单位 m。
 */
float HallEncoder_GetDistance(void);

/**
 * @brief 清零计数、速度和距离状态。
 */
void HallEncoder_Reset(void);

/**
 * @brief 只清零累计距离，不清零编码器原始计数。
 */
void HallEncoder_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* HALL_ENCODER_H */
