/**
 * @file  grayscale_sensor.c
 * @brief BSP 8 路光敏灰度传感器实现（基于感为官方硬件 I2C 例程与 GPIO 兼容）。
 *        与 JY61P 陀螺仪共用硬件 I2C0 (PA0=SDA, PA1=SCL)。
 */
#include "grayscale_sensor.h"
#include "bsp_time.h"
#include <string.h>
#include <stddef.h>

/* 感为 I2C 在线状态、检测到的 I2C 地址与最近一帧原始数据 */
static bool s_gw_i2c_online = false;
static uint8_t s_gw_i2c_addr = GW_GRAY_I2C_ADDR; /* 默认 0x4C */
static uint8_t s_gw_cached_raw = 0xFFU;           /* 默认全白(1) */
static uint32_t s_last_probe_ms = 0U;

#define GW_I2C_TIMEOUT_CYCLES  200000U /* 足够长，保证时钟拉伸与传输不早夭，但遇死锁约 6ms 退出 */

/* 校准缓冲区与基准阈值记录（感为默认或已标定值） */
static uint8_t s_calib_white[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {200, 200, 200, 200, 200, 200, 200, 200};
static uint8_t s_calib_black[GRAYSCALE_SENSOR_CHANNEL_COUNT] = {50, 50, 50, 50, 50, 50, 50, 50};
static uint8_t s_calib_th_w[GRAYSCALE_SENSOR_CHANNEL_COUNT]  = {150, 150, 150, 150, 150, 150, 150, 150};
static uint8_t s_calib_th_b[GRAYSCALE_SENSOR_CHANNEL_COUNT]  = {100, 100, 100, 100, 100, 100, 100, 100};

/* ═══════════════════ 感为官方硬件 IIC 通信底层 (I2C0) ═══════════════════
 * 参考: NUEDC_2026/[Doc] Hardware/感为灰度Keil_MSPM0例程/examples/Hardware_IIC/keil/BSP/IIC.c
 * 严格按照感为官方驱动调用时序，不盲目调用 resetControllerTransfer()，完整配合从机 STM8
 */

static bool Gw_Hardware_IIC_WriteByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t dat)
{
    uint32_t timeout;
    uint8_t temp[2];
    temp[0] = reg_addr;
    temp[1] = dat;

    DL_I2C_fillControllerTXFIFO(Gray_JY61P_I2C_INST, temp, 2U);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2U);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0U) return false;
    }

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);
    return true;
}

static bool Gw_Hardware_IIC_WriteBytes(uint8_t slave_addr, uint8_t reg_addr, const uint8_t *dat, uint8_t len)
{
    uint32_t timeout;
    uint8_t temp[GRAYSCALE_SENSOR_CHANNEL_COUNT + 1U];
    if (dat == NULL || len == 0U || len > GRAYSCALE_SENSOR_CHANNEL_COUNT) {
        return false;
    }

    temp[0] = reg_addr;
    memcpy(temp + 1, dat, len);

    DL_I2C_fillControllerTXFIFO(Gray_JY61P_I2C_INST, temp, (uint32_t)(len + 1U));

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t)(len + 1U));

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0U) return false;
    }

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);
    return true;
}

static bool Gw_Hardware_IIC_ReadByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *val)
{
    uint32_t timeout;

    DL_I2C_fillControllerTXFIFO(Gray_JY61P_I2C_INST, &reg_addr, 1U);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0U) return false;
    }

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);

    if (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return false;
    }

    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 1U);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0U) return false;
    }

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    if (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return false;
    }

    if (val != NULL) {
        *val = DL_I2C_receiveControllerData(Gray_JY61P_I2C_INST);
    }
    return true;
}

static bool Gw_Hardware_IIC_ReadBytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint32_t timeout;
    if (data == NULL || len == 0U) {
        return false;
    }

    DL_I2C_fillControllerTXFIFO(Gray_JY61P_I2C_INST, &reg_addr, 1U);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0U) return false;
    }

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    DL_I2C_flushControllerTXFIFO(Gray_JY61P_I2C_INST);

    if (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return false;
    }

    DL_I2C_startControllerTransfer(Gray_JY61P_I2C_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0U) return false;
    }

    timeout = GW_I2C_TIMEOUT_CYCLES;
    while (!(DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) return false;
    }

    if (DL_I2C_getControllerStatus(Gray_JY61P_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return false;
    }

    for (uint8_t i = 0U; i < len; i++) {
        data[i] = DL_I2C_receiveControllerData(Gray_JY61P_I2C_INST);
    }
    return true;
}

/* ═══════════════════ 感为高级状态机接口 ═══════════════════ */

bool GrayscaleSensor_Probe(void)
{
    uint8_t ping_val = 0U;
    const uint8_t candidate_addrs[] = { GW_GRAY_I2C_ADDR, 0x4D, 0x4E, 0x4F };

    for (uint8_t a = 0U; a < sizeof(candidate_addrs); a++) {
        uint8_t addr = candidate_addrs[a];
        if (Gw_Hardware_IIC_ReadByte(addr, GW_GRAY_REG_PING, &ping_val)) {
            if (ping_val == GW_GRAY_PING_RESPONSE) {
                s_gw_i2c_addr = addr;
                s_gw_i2c_online = true;
                return true;
            }
        }
    }
    s_gw_i2c_online = false;
    return false;
}

void GrayscaleSensor_Init(void)
{
    /* 上电初次探测：重试 5 次，每次延时约 5ms，等待感为从机 STM8 上电初始化完成 */
    for (uint8_t i = 0U; i < 5U; i++) {
        if (GrayscaleSensor_Probe()) {
            break;
        }
        for (volatile uint32_t d = 0U; d < 160000U; d++) { __NOP(); }
    }

    if (s_gw_i2c_online) {
        uint8_t raw = 0xFFU;
        if (Gw_Hardware_IIC_ReadByte(s_gw_i2c_addr, GW_GRAY_REG_DIGITAL, &raw)) {
            s_gw_cached_raw = raw;
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
        uint8_t raw = 0xFFU;
        if (Gw_Hardware_IIC_ReadByte(s_gw_i2c_addr, GW_GRAY_REG_DIGITAL, &raw)) {
            s_gw_cached_raw = raw;
        }
        /* channel 0 -> 探头8 (bit7), channel 7 -> 探头1 (bit0). 0=黑线, 1=白底 */
        return (uint8_t)((s_gw_cached_raw >> (7U - (uint8_t)channel)) & 1U);
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

    /* 若当前未在线，每隔 50ms 尝试一次非阻塞 Ping 探测，实现热插拔与延迟上电自愈 */
    uint32_t now = BSP_Time_GetMs();
    if (!s_gw_i2c_online && (now - s_last_probe_ms >= 50U)) {
        s_last_probe_ms = now;
        (void)GrayscaleSensor_Probe();
    }

    if (s_gw_i2c_online) {
        uint8_t raw = 0xFFU;
        if (Gw_Hardware_IIC_ReadByte(s_gw_i2c_addr, GW_GRAY_REG_DIGITAL, &raw)) {
            s_gw_cached_raw = raw;
        }
        /* 
         * 感为官方协议: bit0=探头1, bit7=探头8。0=黑线, 1=白底。
         * 天猛星小车通道语义:
         *   channel 0 (-3.5, 最左探头) -> 探头 8 (bit7)
         *   channel 7 (+3.5, 最右探头) -> 探头 1 (bit0)
         *   巡线核心约定: 0=黑线, 1=白底。
         * 因此: channel i 对应 bit (7 - i)，直接保持 0=黑 1=白！
         */
        for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
            digital_array[i] = (uint8_t)((s_gw_cached_raw >> (7U - i)) & 1U);
        }
        return;
    }

    /* GPIO 回退逐路读取（未接 I2C 时兼容） */
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        digital_array[i] = GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i);
    }
}

uint8_t GrayscaleSensor_ReadMask(void)
{
    /* 若当前未在线，非阻塞自动重连 */
    uint32_t now = BSP_Time_GetMs();
    if (!s_gw_i2c_online && (now - s_last_probe_ms >= 50U)) {
        s_last_probe_ms = now;
        (void)GrayscaleSensor_Probe();
    }

    if (s_gw_i2c_online) {
        uint8_t raw = 0xFFU;
        if (Gw_Hardware_IIC_ReadByte(s_gw_i2c_addr, GW_GRAY_REG_DIGITAL, &raw)) {
            s_gw_cached_raw = raw;
        }
        /*
         * 上位机 PC 地面站指示灯：bit 为 1 表示检测到黑线（亮绿灯）。
         * 感为硬件 0xDD 返回值中 0=黑线，1=白底。
         * 因此取反 ~s_gw_cached_raw 刚好使黑线处 bit 为 1！
         * bit0~bit7 对应探头 1~8。
         */
        return (uint8_t)(~s_gw_cached_raw);
    }

    /* GPIO 状态组合成掩码（1 表示黑线） */
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        if (GrayscaleSensor_ReadSingle((GRAYSCALE_SENSOR_CHANNEL)i) == 0U) {
            mask |= (uint8_t)(1U << (7U - i));
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
    uint8_t raw[GRAYSCALE_SENSOR_CHANNEL_COUNT];
    if (!Gw_Hardware_IIC_ReadBytes(s_gw_i2c_addr, GW_GRAY_REG_ANALOG_BASE, raw, GRAYSCALE_SENSOR_CHANNEL_COUNT)) {
        return false;
    }
    /*
     * raw[0]=探头1(右) ~ raw[7]=探头8(左)。
     * 映射到 channel 0 (最左) ~ channel 7 (最右):
     */
    for (uint8_t i = 0U; i < GRAYSCALE_SENSOR_CHANNEL_COUNT; i++) {
        analog_array[i] = raw[7U - i];
    }
    return true;
}

bool GrayscaleSensor_CalibrateSampleWhite(uint8_t out_white[GRAYSCALE_SENSOR_CHANNEL_COUNT])
{
    if (!s_gw_i2c_online) {
        if (!GrayscaleSensor_Probe()) return false;
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
        if (!GrayscaleSensor_Probe()) return false;
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
        if (!GrayscaleSensor_Probe()) return false;
    }

    uint8_t send_w[GRAYSCALE_SENSOR_CHANNEL_COUNT];
    uint8_t send_b[GRAYSCALE_SENSOR_CHANNEL_COUNT];

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

        /* 感为官方例程滞回比较阈值公式: (No_Mcu_Ganv_Grayscale_Sensor.c L126-127)
         * Gray_white = (white * 2 + black) / 3;
         * Gray_black = (white + black * 2) / 3;
         */
        s_calib_th_w[i] = (uint8_t)((w * 2U + b) / 3U);
        s_calib_th_b[i] = (uint8_t)((w + b * 2U) / 3U);

        if (out_th_w != NULL) { out_th_w[i] = s_calib_th_w[i]; }
        if (out_th_b != NULL) { out_th_b[i] = s_calib_th_b[i]; }

        /* 发送给传感器时，转换为探头 1(右)~探头 8(左) 顺序 */
        send_w[7U - i] = s_calib_th_w[i];
        send_b[7U - i] = s_calib_th_b[i];
    }

    /* 下发给感为八路灰度硬件寄存器 */
    bool ok_w = Gw_Hardware_IIC_WriteBytes(s_gw_i2c_addr, GW_GRAY_REG_CALIB_WHITE, send_w, GRAYSCALE_SENSOR_CHANNEL_COUNT);
    bool ok_b = Gw_Hardware_IIC_WriteBytes(s_gw_i2c_addr, GW_GRAY_REG_CALIB_BLACK, send_b, GRAYSCALE_SENSOR_CHANNEL_COUNT);

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
