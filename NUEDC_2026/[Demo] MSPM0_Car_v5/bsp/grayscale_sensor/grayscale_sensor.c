/**
 * @file  grayscale_sensor.c
 * @brief BSP 8 路光敏灰度传感器实现（感为 I2C 八路灰度与 GPIO 兼容）。
 */
#include "grayscale_sensor.h"
#include <stddef.h>

/* 感为 I2C 在线状态与最近一帧原始数据 */
static bool s_gw_i2c_online = false;
static uint8_t s_gw_cached_raw = 0x00U;

#define GW_I2C_GUARD_IDLE   20000U
#define GW_I2C_GUARD_XFER   25000U

/**
 * @brief 从感为 I2C 灰度模块读取 1 字节寄存器。
 * @param reg 寄存器地址
 * @param val 输出数值指针
 * @return true 成功，false 失败/超时
 */
static bool Gw_I2C_ReadByte(uint8_t reg, uint8_t *val)
{
    uint32_t guard;
    uint32_t status;

    if (val == NULL) {
        return false;
    }

    /* 1) 等待 I2C0 控制器就绪 */
    guard = GW_I2C_GUARD_IDLE;
    while (((status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST)) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) {
            return false;
        }
    }

    /* 2) 清除并刷新 FIFO */
    DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);
    DL_I2C_flushControllerRXFIFO(Gray_JY61P_I2C_INST);

    /* 3) TX 阶段：发送目标寄存器地址，抑制 STOP 准备重复起始 */
    DL_I2C_transmitControllerData(Gray_JY61P_I2C_INST, reg);
    DL_I2C_startControllerTransferAdvanced(Gray_JY61P_I2C_INST, GW_GRAY_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);

    /* 4) 等待 BUSY 置位 */
    guard = 200U;
    while (((status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST)) &
            DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
        if (--guard == 0U) { break; }
    }

    /* 5) 等待地址发送完成 */
    guard = GW_I2C_GUARD_XFER;
    while (((status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST)) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
        return false;
    }

    /* 6) RX 阶段：重复起始读 1 字节数据，使能 STOP */
    DL_I2C_startControllerTransferAdvanced(Gray_JY61P_I2C_INST, GW_GRAY_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_ENABLE);

    /* 7) 等待 BUSY 置位 */
    guard = 200U;
    while (((status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST)) &
            DL_I2C_CONTROLLER_STATUS_BUSY) == 0U) {
        if (--guard == 0U) { break; }
    }

    /* 8) 等待接收完成 */
    guard = GW_I2C_GUARD_XFER;
    while (((status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST)) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
        return false;
    }

    /* 9) 从 RX FIFO 取出数据 */
    guard = 2000U;
    while (DL_I2C_isControllerRXFIFOEmpty(Gray_JY61P_I2C_INST)) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    *val = DL_I2C_receiveControllerData(Gray_JY61P_I2C_INST);
    return true;
}

void GrayscaleSensor_Init(void)
{
    uint8_t ping_val = 0U;
    s_gw_i2c_online = false;

    /* Ping 探查感为 I2C 从机（返回 0x66 表示正常） */
    for (uint8_t retry = 0; retry < 3U; retry++) {
        if (Gw_I2C_ReadByte(GW_GRAY_REG_PING, &ping_val)) {
            if (ping_val == GW_GRAY_PING_RESPONSE) {
                s_gw_i2c_online = true;
                break;
            }
        }
    }

    /* 若 Ping 成功，预读一次 8 路数字量 */
    if (s_gw_i2c_online) {
        uint8_t dummy = 0U;
        if (Gw_I2C_ReadByte(GW_GRAY_REG_DIGITAL, &dummy)) {
            s_gw_cached_raw = dummy;
        }
    }
}

bool GrayscaleSensor_IsI2cOnline(void)
{
    return s_gw_i2c_online;
}

/* ═══════════════ GPIO 回退结构（未接 I2C 时的兼容） ═══════════════ */
typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} GRAYSCALE_SENSOR_HW_CONFIG;

static const GRAYSCALE_SENSOR_HW_CONFIG s_grayscale_sensor_hw[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {
    [GRAYSCALE_SENSOR_CHANNEL_0] = { .port = GRAYSCALE_SENSOR_8_PORT, .pin = GRAYSCALE_SENSOR_8_PIN },
    [GRAYSCALE_SENSOR_CHANNEL_1] = { .port = GRAYSCALE_SENSOR_7_PORT, .pin = GRAYSCALE_SENSOR_7_PIN },
    [GRAYSCALE_SENSOR_CHANNEL_2] = { .port = GRAYSCALE_SENSOR_6_PORT, .pin = GRAYSCALE_SENSOR_6_PIN },
    [GRAYSCALE_SENSOR_CHANNEL_3] = { .port = GRAYSCALE_SENSOR_5_PORT, .pin = GRAYSCALE_SENSOR_5_PIN },
    [GRAYSCALE_SENSOR_CHANNEL_4] = { .port = GRAYSCALE_SENSOR_4_PORT, .pin = GRAYSCALE_SENSOR_4_PIN },
    [GRAYSCALE_SENSOR_CHANNEL_5] = { .port = GRAYSCALE_SENSOR_3_PORT, .pin = GRAYSCALE_SENSOR_3_PIN },
    [GRAYSCALE_SENSOR_CHANNEL_6] = { .port = GRAYSCALE_SENSOR_2_PORT, .pin = GRAYSCALE_SENSOR_2_PIN },
    [GRAYSCALE_SENSOR_CHANNEL_7] = { .port = GRAYSCALE_SENSOR_1_PORT, .pin = GRAYSCALE_SENSOR_1_PIN },
};

uint8_t GrayscaleSensor_ReadSingle(GRAYSCALE_SENSOR_CHANNEL channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNEL_MAX) {
        return 1U; /* 默认白底 */
    }

    if (s_gw_i2c_online) {
        uint8_t raw = 0U;
        if (Gw_I2C_ReadByte(GW_GRAY_REG_DIGITAL, &raw)) {
            s_gw_cached_raw = raw;
        }
        /* 感为模块：bit 为 1 表示黑线；巡线核心要求：0 表示黑线，1 表示白底 */
        uint8_t is_black = ((s_gw_cached_raw >> (uint8_t)channel) & 1U);
        return is_black ? 0U : 1U;
    }

    /* GPIO 回退读取 */
    const GRAYSCALE_SENSOR_HW_CONFIG *hw = &s_grayscale_sensor_hw[channel];
    uint8_t high_level = (DL_GPIO_readPins(hw->port, hw->pin) != 0U) ? 1U : 0U;
    if (GRAYSCALE_SENSOR_ACTIVE_LOW != 0U) {
        /* 低电平为黑线，返回 0 表示黑线 */
        return high_level;
    }
    return (!high_level);
}

void GrayscaleSensor_Read(uint8_t digital_array[GRAYSCALE_SENSOR_CHANNEL_COUNT])
{
    if (digital_array == NULL) {
        return;
    }

    if (s_gw_i2c_online) {
        uint8_t raw = 0U;
        if (Gw_I2C_ReadByte(GW_GRAY_REG_DIGITAL, &raw)) {
            s_gw_cached_raw = raw;
        }
        /* 探头 1~8 映射到 channel 0~7：黑线置 0U，白底置 1U */
        for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
            uint8_t is_black = ((s_gw_cached_raw >> i) & 1U);
            digital_array[i] = is_black ? 0U : 1U;
        }
        return;
    }

    /* GPIO 回退逐路读取 */
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        digital_array[i] = GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i);
    }
}

uint8_t GrayscaleSensor_ReadMask(void)
{
    if (s_gw_i2c_online) {
        uint8_t raw = 0U;
        if (Gw_I2C_ReadByte(GW_GRAY_REG_DIGITAL, &raw)) {
            s_gw_cached_raw = raw;
        }
        /* 感为原始 bit0~7 即对应 1~8 路黑线指示（1=黑线，0=白底） */
        return s_gw_cached_raw;
    }

    /* GPIO 状态组合成掩码（1 表示黑线） */
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        if (GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i) == 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}
