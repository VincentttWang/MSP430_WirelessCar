/**
 * @file  grayscale_sensor.c
 * @brief BSP 8 路光敏灰度传感器实现（感为 I2C 八路灰度与 GPIO 兼容）。
 */
#include "grayscale_sensor.h"
#include <string.h>
#include <stddef.h>

/* 感为 I2C 在线状态、检测到的 I2C 地址与最近一帧原始数据 */
static bool s_gw_i2c_online = false;
static uint8_t s_gw_i2c_addr = GW_GRAY_I2C_ADDR; /* 默认 0x4C */
static uint8_t s_gw_cached_raw = 0x00U;

#define GW_I2C_GUARD_IDLE   15000U
#define GW_I2C_GUARD_XFER   25000U

/* 校准缓冲区与基准阈值记录（感为默认或已标定值） */
static uint8_t s_calib_white[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {200, 200, 200, 200, 200, 200, 200, 200};
static uint8_t s_calib_black[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {50, 50, 50, 50, 50, 50, 50, 50};
static uint8_t s_calib_th_w[GRAYSCALE_SENSOR_CHANNEL_COUNT]  = {150, 150, 150, 150, 150, 150, 150, 150};
static uint8_t s_calib_th_b[GRAYSCALE_SENSOR_CHANNEL_COUNT]  = {100, 100, 100, 100, 100, 100, 100, 100};

/**
 * @brief I2C0 总线死锁解脱（9 个时钟脉冲 + 停止位）
 */
static void Gw_I2C_BusRecover(void)
{
    DL_I2C_disableController(Gray_JY61P_I2C_INST);

    DL_GPIO_initDigitalOutput(GPIO_Gray_JY61P_I2C_IOMUX_SCL);
    DL_GPIO_initDigitalOutput(GPIO_Gray_JY61P_I2C_IOMUX_SDA);
    DL_GPIO_setPins(GPIO_Gray_JY61P_I2C_SCL_PORT, GPIO_Gray_JY61P_I2C_SCL_PIN);
    DL_GPIO_setPins(GPIO_Gray_JY61P_I2C_SDA_PORT, GPIO_Gray_JY61P_I2C_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_Gray_JY61P_I2C_SCL_PORT, GPIO_Gray_JY61P_I2C_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_Gray_JY61P_I2C_SDA_PORT, GPIO_Gray_JY61P_I2C_SDA_PIN);
    DL_Common_delayCycles(320U);

    for (uint8_t i = 0U; i < 9U; i++) {
        DL_GPIO_clearPins(GPIO_Gray_JY61P_I2C_SCL_PORT, GPIO_Gray_JY61P_I2C_SCL_PIN);
        DL_Common_delayCycles(320U);
        DL_GPIO_setPins(GPIO_Gray_JY61P_I2C_SCL_PORT, GPIO_Gray_JY61P_I2C_SCL_PIN);
        DL_Common_delayCycles(320U);
    }

    DL_GPIO_clearPins(GPIO_Gray_JY61P_I2C_SDA_PORT, GPIO_Gray_JY61P_I2C_SDA_PIN);
    DL_Common_delayCycles(320U);
    DL_GPIO_setPins(GPIO_Gray_JY61P_I2C_SCL_PORT, GPIO_Gray_JY61P_I2C_SCL_PIN);
    DL_Common_delayCycles(320U);
    DL_GPIO_setPins(GPIO_Gray_JY61P_I2C_SDA_PORT, GPIO_Gray_JY61P_I2C_SDA_PIN);
    DL_Common_delayCycles(320U);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_Gray_JY61P_I2C_IOMUX_SDA,
        GPIO_Gray_JY61P_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_Gray_JY61P_I2C_IOMUX_SCL,
        GPIO_Gray_JY61P_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_Gray_JY61P_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_Gray_JY61P_I2C_IOMUX_SCL);

    DL_I2C_enableController(Gray_JY61P_I2C_INST);
}

static bool Gw_I2C_ReadBytesEx(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t len)
{
    uint32_t guard;
    uint32_t status;

    if (val == NULL || len == 0U) {
        return false;
    }

    /* 若总线被从机卡在 BUSY 状态，执行总线恢复 */
    if ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        Gw_I2C_BusRecover();
    }

    /* 1) 等待 I2C0 控制器就绪 */
    guard = GW_I2C_GUARD_IDLE;
    while (((status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST)) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) {
            Gw_I2C_BusRecover();
            return false;
        }
    }

    /* 2) 填充目标寄存器地址至 TX FIFO */
    DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);
    DL_I2C_flushControllerRXFIFO(Gray_JY61P_I2C_INST);

    DL_I2C_fillControllerTXFIFO(Gray_JY61P_I2C_INST, &reg, 1U);

    guard = GW_I2C_GUARD_IDLE;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    /* 3) TX 阶段：发送 1 字节寄存器地址（产生完整 START + STOP，感为 STM8 固件需要 STOP 触发地址锁存） */
    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    guard = GW_I2C_GUARD_XFER;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    guard = GW_I2C_GUARD_IDLE;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST);
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
        return false;
    }
    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);

    /* 4) RX 阶段：读取 len 字节数据 */
    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, addr,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, (uint32_t)len);

    guard = GW_I2C_GUARD_XFER;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    guard = GW_I2C_GUARD_IDLE;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST);
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
        return false;
    }

    for (uint8_t i = 0U; i < len; i++) {
        val[i] = DL_I2C_receiveControllerData(Gray_JY61P_I2C_INST);
    }
    return true;
}

static bool Gw_I2C_ReadBytes(uint8_t reg, uint8_t *val, uint8_t len)
{
    return Gw_I2C_ReadBytesEx(s_gw_i2c_addr, reg, val, len);
}

static bool Gw_I2C_ReadByte(uint8_t reg, uint8_t *val)
{
    return Gw_I2C_ReadBytes(reg, val, 1U);
}

static bool Gw_I2C_WriteBytes(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint32_t guard;
    uint32_t status;
    uint8_t temp[GRAYSCALE_SENSOR_CHANNEL_COUNT + 1U];

    if (data == NULL || len == 0U || len > GRAYSCALE_SENSOR_CHANNEL_COUNT) {
        return false;
    }

    guard = GW_I2C_GUARD_IDLE;
    while (((status = DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST)) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) return false;
    }

    DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);
    DL_I2C_flushControllerRXFIFO(Gray_JY61P_I2C_INST);

    temp[0] = reg;
    memcpy(&temp[1], data, len);

    DL_I2C_fillControllerTXFIFO(Gray_JY61P_I2C_INST, temp, (uint32_t)(len + 1U));

    guard = GW_I2C_GUARD_IDLE;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, s_gw_i2c_addr,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)(len + 1U));

    guard = GW_I2C_GUARD_XFER;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }

    guard = GW_I2C_GUARD_IDLE;
    while ((DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) {
            DL_I2C_resetControllerTransfer(Gray_JY61P_I2C_INST);
            return false;
        }
    }
    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);
    return true;
}

bool GrayscaleSensor_Probe(void)
{
    uint8_t ping_val = 0U;
    const uint8_t candidate_addrs[] = { GW_GRAY_I2C_ADDR, 0x4D, 0x4E, 0x4F };

    s_gw_i2c_online = false;

    /* 自动轮询扫描可能配置的 AD0/AD1 地址 (0x4C ~ 0x4F) */
    for (uint8_t a = 0U; a < sizeof(candidate_addrs); a++) {
        uint8_t addr = candidate_addrs[a];
        for (uint8_t retry = 0U; retry < 3U; retry++) {
            if (Gw_I2C_ReadBytesEx(addr, GW_GRAY_REG_PING, &ping_val, 1U)) {
                if (ping_val == GW_GRAY_PING_RESPONSE) {
                    s_gw_i2c_addr = addr;
                    s_gw_i2c_online = true;
                    return true;
                }
            }
            for (volatile uint32_t d = 0U; d < 8000U; d++) { __NOP(); }
        }
    }
    return false;
}

void GrayscaleSensor_Init(void)
{
    (void)GrayscaleSensor_Probe();

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

uint8_t GrayscaleSensor_GetI2cAddr(void)
{
    return s_gw_i2c_addr;
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

bool GrayscaleSensor_ReadAnalog(uint8_t analog_array[GRAYSCALE_SENSOR_CHANNEL_COUNT])
{
    if (analog_array == NULL) {
        return false;
    }
    if (!s_gw_i2c_online) {
        if (!GrayscaleSensor_Probe()) {
            return false;
        }
    }
    return Gw_I2C_ReadBytes(GW_GRAY_REG_ANALOG_BASE, analog_array, GRAYSCALE_SENSOR_CHANNEL_COUNT);
}

bool GrayscaleSensor_CalibrateSampleWhite(uint8_t out_white[GRAYSCALE_SENSOR_CHANNEL_COUNT])
{
    if (!s_gw_i2c_online) {
        return false;
    }

    uint32_t sum[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {0};
    uint8_t raw[GRAYSCALE_SENSOR_CHANNEL_COUNT];
    const uint8_t SAMPLES = 8U;

    for (uint8_t s = 0U; s < SAMPLES; s++) {
        if (!GrayscaleSensor_ReadAnalog(raw)) {
            return false;
        }
        for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
            sum[i] += raw[i];
        }
        for (volatile uint32_t d = 0U; d < 8000U; d++) { __NOP(); }
    }

    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        s_calib_white[i] = (uint8_t)(sum[i] / SAMPLES);
        if (out_white != NULL) {
            out_white[i] = s_calib_white[i];
        }
    }
    return true;
}

bool GrayscaleSensor_CalibrateSampleBlack(uint8_t out_black[GRAYSCALE_SENSOR_CHANNEL_COUNT])
{
    if (!s_gw_i2c_online) {
        return false;
    }

    uint32_t sum[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {0};
    uint8_t raw[GRAYSCALE_SENSOR_CHANNEL_COUNT];
    const uint8_t SAMPLES = 8U;

    for (uint8_t s = 0U; s < SAMPLES; s++) {
        if (!GrayscaleSensor_ReadAnalog(raw)) {
            return false;
        }
        for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
            sum[i] += raw[i];
        }
        for (volatile uint32_t d = 0U; d < 8000U; d++) { __NOP(); }
    }

    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        s_calib_black[i] = (uint8_t)(sum[i] / SAMPLES);
        if (out_black != NULL) {
            out_black[i] = s_calib_black[i];
        }
    }
    return true;
}

bool GrayscaleSensor_ApplyCalibration(uint8_t out_th_w[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                      uint8_t out_th_b[GRAYSCALE_SENSOR_CHANNEL_COUNT])
{
    if (!s_gw_i2c_online) {
        return false;
    }

    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        uint16_t w = s_calib_white[i];
        uint16_t b = s_calib_black[i];

        if (w == b) {
            if (w >= 128U) { b = (uint16_t)(w - 50U); }
            else           { w = (uint16_t)(b + 50U); }
        }
        if (b > w) {
            uint16_t temp = w; w = b; b = temp;
        }

        /* 感为官方 Keil 例程滞回阈值公式: (No_Mcu_Ganv_Grayscale_Sensor.c L126-127)
         * Gray_white = (white * 2 + black) / 3;
         * Gray_black = (white + black * 2) / 3;
         */
        s_calib_th_w[i] = (uint8_t)((w * 2U + b) / 3U);
        s_calib_th_b[i] = (uint8_t)((w + b * 2U) / 3U);

        if (out_th_w != NULL) { out_th_w[i] = s_calib_th_w[i]; }
        if (out_th_b != NULL) { out_th_b[i] = s_calib_th_b[i]; }
    }

    /* 下发给感为八路灰度硬件寄存器 */
    bool ok_w = Gw_I2C_WriteBytes(GW_GRAY_REG_CALIB_WHITE, s_calib_th_w, GRAYSCALE_SENSOR_CHANNEL_COUNT);
    bool ok_b = Gw_I2C_WriteBytes(GW_GRAY_REG_CALIB_BLACK, s_calib_th_b, GRAYSCALE_SENSOR_CHANNEL_COUNT);

    return (ok_w && ok_b);
}

void GrayscaleSensor_GetCalibrationData(uint8_t white[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                       uint8_t black[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                       uint8_t th_white[GRAYSCALE_SENSOR_CHANNEL_COUNT],
                                       uint8_t th_black[GRAYSCALE_SENSOR_CHANNEL_COUNT])
{
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        if (white != NULL)    white[i]    = s_calib_white[i];
        if (black != NULL)    black[i]    = s_calib_black[i];
        if (th_white != NULL) th_white[i] = s_calib_th_w[i];
        if (th_black != NULL) th_black[i] = s_calib_th_b[i];
    }
}
