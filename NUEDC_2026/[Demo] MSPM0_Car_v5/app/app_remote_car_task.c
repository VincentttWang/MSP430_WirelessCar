/**
 * @file  app_remote_car_task.c
 * @brief MSPM0G3507 智能小车双模遥控与 MSP430 协同任务实现
 */
#include "app_remote_car_task.h"

#include "bsp_time.h"
#include "canmv_uart.h"
#include "chassis.h"
#include "key.h"
#include "oled.h"
#include "ti_msp_dl_config.h"
#include "ui.h"
#include "wit_sdk.h"
#include "grayscale_sensor.h"
#include "line_tracking.h"
#include "line_follow.h"
#include "pid.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define RC_TELEMETRY_PERIOD_MS   50U     /* 20Hz 遥测帧周期 */
#define RC_WATCHDOG_TIMEOUT_MS   1200U   /* 指令超时自动刹车保护 */
#define RC_UI_REFRESH_PERIOD_MS  100U    /* OLED 刷新周期 */
#define RC_CMD_BUFFER_LEN        32U     /* 指令缓存长度 */

/* 方向指令"失联"超时（死手开关）。
 *
 * 上位机按住 WASD 时每 200ms 补发一次；一旦松手 / 丢包，这里 400ms 内
 * 必须把电机停掉，而不是继续跑 1.2s 才被总看门狗刹停。
 * 400ms = 200ms 补发周期 + 200ms 余量，正常连发时不会被误判。
 *
 * 另一个作用：RUN 只是"使能"，本身不含任何运动意图；
 * 任何残留的方向指令超过这个时间都必须失效，车才能停在原地等操作。 */
#define RC_MOTION_TIMEOUT_MS     400U

/* ── 方向位打包（与上位机 index.html 的 DIR_BIT 一一对应）───────────────
 * 旧协议一次只发一个字符 'w'/'a'，一个字节只能表达一个方向，
 * 按第二个键就把第一个**覆盖**掉 —— 所以永远走不出斜线。
 * 改成掩码后，前后分量和左右分量可以同时存在、独立叠加。 */
#define RC_DIR_FWD       0x01U
#define RC_DIR_REV       0x02U
#define RC_DIR_LEFT      0x04U
#define RC_DIR_RIGHT     0x08U
#define RC_DIR_MASK_ALL  0x0FU

/* 直行 / 转向分量（占空比 %，还要再乘限速系数）。
 * 两者独立叠加：直行 50 + 转向 20 -> 斜向就是 70/30，
 * 有明显弧线又不会变成原地急转。 */
#define RC_BASE_DUTY_PCT 50.0f
#define RC_TURN_DUTY_PCT 20.0f

/* 转向极性（+1 / -1）。
 * 实测：按"左轮减速、右轮加速"的正规差速算出来的输出，车实际向右转，
 * 说明两路电机在车上的左右位置与通道命名相反。
 * 取 +1 时，A 键输出 (left=+turn, right=-turn)，实测为左转。
 * 以后若重装电机发现又反了，只改这一个值即可，别去动上面的分量。 */
#define RC_STEER_SIGN    (+1.0f)

/* 遥控与运行状态 */
static volatile uint8_t s_speed_pct = 50U;     /* 默认限速 50% */
static volatile uint8_t s_car_state = 0U;      /* 0: STOP, 1: RUN, 3: BRAKE */
static volatile uint8_t s_traffic_light = 0U;  /* 0: RED, 1: GREEN */
static volatile uint32_t s_last_cmd_ms = 0U;
/* 最后一次"方向类"指令的时刻。心跳/调速包都不刷新它，
 * 所以它是判断"现在到底该不该出力"的唯一依据（见 RC_MOTION_TIMEOUT_MS）。 */
static volatile uint32_t s_last_motion_ms = 0U;
/* 当前是否有生效中的方向指令。用于"只降一次"地收尾，
 * 避免在主循环里每 5ms 重复刹车、重复改 ACT 文本。 */
static volatile bool s_motion_active = false;
/* 最近一帧方向掩码。调速时用它重算一次输出，不需要再靠比较字符串，
 * 也天然支持"斜向 + 调速"这种组合。 */
static volatile uint8_t s_last_dir_mask = 0U;
static volatile char s_last_action[16] = "STOP";
static volatile char s_prev_action[16] = "STOP";
static volatile uint32_t s_reset_action_ms = 0U;
static volatile float s_yaw_zero_offset = 0.0f;
static volatile float s_gravity_ref = 1.00f;   /* 静止重力矢量模长基准 (用于解算动态净加速度并静止归零) */

/* ── 感为八路灰度传感器校准状态机 ── */
typedef enum {
    CALIB_STATE_IDLE = 0,
    CALIB_STATE_WAIT_WHITE,
    CALIB_STATE_WAIT_BLACK,
    CALIB_STATE_DONE
} SENSOR_CALIB_STATE;

static volatile SENSOR_CALIB_STATE s_calib_state = CALIB_STATE_IDLE;
static volatile uint32_t s_calib_done_ms = 0U;

static void AppRemoteCar_SendPacket(const char *pkt) {
    if (pkt == NULL) return;
    for (uint16_t i = 0U; pkt[i] != '\0'; i++) {
        DL_UART_Main_transmitDataBlocking(Wireless_INST, (uint8_t)pkt[i]);
        DL_UART_Main_transmitDataBlocking(Debug_INST, (uint8_t)pkt[i]);
    }
}

static void AppRemoteCar_Beep(uint16_t duration_ms) {
    DL_GPIO_setPins(Buzzer_PORT, Buzzer_PIN);
    for (volatile uint32_t i = 0; i < (uint32_t)duration_ms * 3200U; i++) {
        __NOP();
    }
    DL_GPIO_clearPins(Buzzer_PORT, Buzzer_PIN);
}

static void AppRemoteCar_HandleDownKey(void) {
    char pkt[128];
    uint8_t buf[GRAYSCALE_SENSOR_CHANNEL_COUNT];
    uint8_t th_w[GRAYSCALE_SENSOR_CHANNEL_COUNT];
    uint8_t th_b[GRAYSCALE_SENSOR_CHANNEL_COUNT];

    if (s_calib_state == CALIB_STATE_IDLE) {
        /* 进入校准模式：待采集白底 */
        s_calib_state = CALIB_STATE_WAIT_WHITE;
        s_car_state = 0U;
        s_motion_active = false;
        strncpy((char*)s_last_action, "CALIB", sizeof(s_last_action));
        (void)Chassis_Brake();
        AppRemoteCar_Beep(60);
        AppRemoteCar_SendPacket("!CAL:WAIT_WHITE#");
    } else if (s_calib_state == CALIB_STATE_WAIT_WHITE) {
        /* 采集白底 */
        if (GrayscaleSensor_CalibrateSampleWhite(buf)) {
            AppRemoteCar_Beep(60);
            snprintf(pkt, sizeof(pkt), "!CAL:W_OK,%u,%u,%u,%u,%u,%u,%u,%u#",
                     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
            AppRemoteCar_SendPacket(pkt);
            s_calib_state = CALIB_STATE_WAIT_BLACK;
        } else {
            AppRemoteCar_Beep(200);
            AppRemoteCar_SendPacket("!CAL:ERR_I2C#");
        }
    } else if (s_calib_state == CALIB_STATE_WAIT_BLACK) {
        /* 采集黑线并生效滞回阈值 */
        if (GrayscaleSensor_CalibrateSampleBlack(buf) &&
            GrayscaleSensor_ApplyCalibration(th_w, th_b)) {
            AppRemoteCar_Beep(60);
            for (volatile uint32_t d = 0; d < 200000; d++) { __NOP(); }
            AppRemoteCar_Beep(60);

            snprintf(pkt, sizeof(pkt), "!CAL:DONE,%u,%u,%u,%u,%u,%u,%u,%u#",
                     th_w[0], th_w[1], th_w[2], th_w[3], th_w[4], th_w[5], th_w[6], th_w[7]);
            AppRemoteCar_SendPacket(pkt);

            s_calib_state = CALIB_STATE_DONE;
            s_calib_done_ms = BSP_Time_GetMs();
        } else {
            AppRemoteCar_Beep(200);
            AppRemoteCar_SendPacket("!CAL:ERR_I2C#");
        }
    } else if (s_calib_state == CALIB_STATE_DONE) {
        s_calib_state = CALIB_STATE_IDLE;
        strncpy((char*)s_last_action, "STOP", sizeof(s_last_action));
        AppRemoteCar_SendPacket("!CAL:EXIT#");
    }
}


/* 指令缓冲区：每个字节来源独立一套。
 * 旧实现三路串口共用一个 s_cmd_idx，只要 PC 与 MSP430 同时说话，
 * '!...# ' 封包就会被对方的字节拆坏 —— 表现为"偶尔能收到、多数丢指令"。 */
#define RC_SRC_CANMV    0U
#define RC_SRC_PC       1U
#define RC_SRC_MSP430   2U
#define RC_SRC_COUNT    3U

typedef struct {
    char    buf[RC_CMD_BUFFER_LEN];
    uint8_t idx;
} RC_RX_PARSER;

static RC_RX_PARSER s_rx_parser[RC_SRC_COUNT];
/* 只有"无线模块"收到字节才算上位机在线；MSP430 的心跳不能点亮连接指示灯 */
static volatile uint32_t s_last_pc_ms = 0U;
/* MSP430 显示终端的最后活动时刻：单独用黄灯指示，与上位机连接状态互不干扰。
 * 这样即使 MSP430 自己黑屏，也能从这盏灯判断它到底有没有在跑。 */
static volatile uint32_t s_last_msp_ms = 0U;

/* 上位机(无线)在线判据：800ms 内收到过 UART1 的字节。
 * 只在这一处定义，三色灯和遥测帧里的 PC 在线标志共用，
 * 避免"灯说连上、仪表说 DISC"这种两个指示器各算一套的问题。 */
/* ── 直行双轮同步 (航向锁) 与编码器闭环控速控制器 ── */
#define RC_MAX_SPEED_MPS         0.85f   /* 100% 满速理论线速度 ~0.85 m/s */
#define RC_TURN_SETTLE_MS        250U    /* 暂停转弯后底盘旋转惯性释放与姿态平稳窗口 (250ms) */

static PID_CONTROLLER s_straight_sync_pid;
static PID_CONTROLLER s_speed_sync_pid;
static volatile bool  s_straight_lock_active = false;
static volatile float s_straight_target_yaw  = 0.0f;
static volatile uint32_t s_last_turn_ms      = 0U;   /* 最后一次存在转弯动作 (A/D/弧线) 的时刻 */
static volatile uint32_t s_straight_start_ms = 0U;   /* 本次直行点动开始时刻 (用于 45ms 起步静摩擦前馈) */

static void AppRemoteCar_InitControllers(void){
    PID_CONFIG straight_cfg = {
        .kp = 2.0f,          /* 比例增益：小角度偏差产生敏锐纠偏力矩 */
        .ki = 0.06f,         /* 积分增益：消除左右电机固有装配摩擦不对称 */
        .kd = 0.04f,         /* 微分增益 */
        .integral_limit = 8.0f,
        .output_limit = 22.0f,
        .mode = PID_MODE_POSITION
    };
    PID_Init(&s_straight_sync_pid, &straight_cfg);

    PID_CONFIG speed_cfg = {
        .kp = 1.5f,
        .ki = 0.2f,
        .kd = 0.0f,
        .integral_limit = 4.0f,
        .output_limit = 5.0f,
        .mode = PID_MODE_POSITION
    };
    PID_Init(&s_speed_sync_pid, &speed_cfg);

    s_straight_lock_active = false;
    s_straight_target_yaw = 0.0f;
}

#define RC_PC_ONLINE_TIMEOUT_MS  800U
#define RC_FLAG_PC_ONLINE        0x80U

static bool AppRemoteCar_IsPcOnline(void){
    uint32_t now = BSP_Time_GetMs();
    return (s_last_pc_ms > 0U) && ((now - s_last_pc_ms) < RC_PC_ONLINE_TIMEOUT_MS);
}

/* 速度输出下限：最低限速下调至 5%（替换原 30% 限制），最高速 100% 满速。 */
#define RC_DUTY_FLOOR_PCT   5.0f

static void AppRemoteCar_Drive(float left, float right){
    if (left > 0.01f && left < RC_DUTY_FLOOR_PCT){
        left = RC_DUTY_FLOOR_PCT;
    } else if (left < -0.01f && left > -RC_DUTY_FLOOR_PCT){
        left = -RC_DUTY_FLOOR_PCT;
    }
    if (right > 0.01f && right < RC_DUTY_FLOOR_PCT){
        right = RC_DUTY_FLOOR_PCT;
    } else if (right < -0.01f && right > -RC_DUTY_FLOOR_PCT){
        right = -RC_DUTY_FLOOR_PCT;
    }
    (void)Chassis_SetDuty(left, right);
}

/* SPD -> 占空比缩放系数 (5% ~ 100%) */
static float AppRemoteCar_SpeedRatio(void){
    float pct = (float)s_speed_pct;
    if (pct < RC_DUTY_FLOOR_PCT){
        pct = RC_DUTY_FLOOR_PCT;
    }
    if (pct > 100.0f){
        pct = 100.0f;
    }
    return pct / 100.0f;
}

/* 直行两轮闭环同步与编码器控速步进函数 */
static void AppRemoteCar_StraightSyncStep(float dt_s){
    if (!s_straight_lock_active){
        return;
    }

    /* 安全防线：如果存在左右转向分量，绝不执行直行差速纠偏，彻底杜绝转弯时对抗摆头 */
    if (s_last_dir_mask & (RC_DIR_LEFT | RC_DIR_RIGHT)){
        s_straight_lock_active = false;
        return;
    }

    float v = 0.0f;
    if (s_last_dir_mask & RC_DIR_FWD) { v += 1.0f; }
    if (s_last_dir_mask & RC_DIR_REV) { v -= 1.0f; }
    if (fabsf(v) < 0.1f){
        return;
    }

    WIT_IMU_DATA imu = {0};
    (void)WitGetData(&imu);

    /* 计算航向误差（相对直行锁定角），范围归一化至 [-180, +180] */
    float yaw_err = imu.attitude_deg.yaw - s_straight_target_yaw;
    while (yaw_err > 180.0f)  yaw_err -= 360.0f;
    while (yaw_err < -180.0f) yaw_err += 360.0f;

    /* 超敏连续微死区：0.05°（对应轮距 150mm 下两轮位移仅 0.13mm）
     * 消除原先 0.5° 大死区导致的短按微幅运动下完全不纠偏的缺陷 */
    float eff_err = 0.0f;
    if (yaw_err > 0.05f){
        eff_err = yaw_err - 0.05f;
    } else if (yaw_err < -0.05f){
        eff_err = yaw_err + 0.05f;
    }

    /* 航向锁定 PID 计算左右两轮差速纠偏量 */
    float steer_corr = PID_Update(&s_straight_sync_pid, eff_err, 0.0f, dt_s);

    /* 融合陀螺仪 Z 轴角速度直接微分阻尼 (Gyro Rate Damping)：
     * 在一侧轮速稍快导致车身开始转动的最初 10ms 立即施加反向力矩，
     * 彻底消除短按起步时的角加速度与两轮位移不均 */
    float gyro_z = imu.gyro_deg_s.z;
    if (fabsf(gyro_z) > 0.3f){
        steer_corr += gyro_z * 0.06f;
    }

    /* 限制纠偏量上限 (±22%) */
    if (steer_corr > 22.0f)  { steer_corr = 22.0f; }
    if (steer_corr < -22.0f) { steer_corr = -22.0f; }

    /* 编码器闭环线速度辅助调控 (平滑低增益，避免 100ms 采样阶跃冲量) */
    float spd_setting = (float)s_speed_pct;
    if (spd_setting < RC_DUTY_FLOOR_PCT) { spd_setting = RC_DUTY_FLOOR_PCT; }
    if (spd_setting > 100.0f) { spd_setting = 100.0f; }

    float target_mps = (spd_setting / 100.0f) * RC_MAX_SPEED_MPS;
    float meas_mps = fabsf(Chassis_GetSpeed());
    float speed_trim = PID_Update(&s_speed_sync_pid, target_mps, meas_mps, dt_s);

    float base_duty = v * spd_setting + v * speed_trim;

    /* 起步静摩擦力前馈 (Stiction Breakaway Boost)：
     * 短按起步最初 45ms 内，若设定速度极低 (如 5%)，临时提升至 18% 以克服齿轮箱静摩擦，
     * 确保左右两电机在按键按下的瞬间同时破阻起动，消除"一侧已走、另一侧迟滞" */
    uint32_t now_ms = BSP_Time_GetMs();
    if ((now_ms - s_straight_start_ms) < 45U && fabsf(base_duty) < 18.0f){
        base_duty = (v > 0.0f) ? 18.0f : -18.0f;
    }

    /* 左右轮对称分配：左轮 +corr，右轮 -corr */
    float l = base_duty + (steer_corr * RC_STEER_SIGN);
    float r = base_duty - (steer_corr * RC_STEER_SIGN);

    if (l >  100.0f) { l =  100.0f; }
    if (l < -100.0f) { l = -100.0f; }
    if (r >  100.0f) { r =  100.0f; }
    if (r < -100.0f) { r = -100.0f; }

    AppRemoteCar_Drive(l, r);
}

/* 方向掩码 -> 左右轮占空比动力解算与分配
 *
 * 1. 纯直行 (W / S): 自动锁定当前航向角，启动双轮闭环同步与编码器控速，杜绝跑偏与两轮失调
 * 2. 原地旋转 (A / D): 旋转速度严格与当前设定速度成正比 (0.70x 系数)，从 5% 到 100% 平滑可控，不再急转暴冲
 * 3. 行进中转向 (W+A / W+D / S+A / S+D): 差速比例与当前 SPD 保持严格线性对应 (45% 差速)，不同速度下保持相同转弯半径与平滑轨迹
 */
static void AppRemoteCar_MixAndDrive(uint8_t mask){
    float v = 0.0f;
    float w = 0.0f;
    float speed;

    if (mask & RC_DIR_FWD)   { v += 1.0f; }
    if (mask & RC_DIR_REV)   { v -= 1.0f; }
    if (mask & RC_DIR_LEFT)  { w += 1.0f; }
    if (mask & RC_DIR_RIGHT) { w -= 1.0f; }

    speed = (float)s_speed_pct;
    if (speed < RC_DUTY_FLOOR_PCT) { speed = RC_DUTY_FLOOR_PCT; }
    if (speed > 100.0f) { speed = 100.0f; }

    /* 情况 1: 松手或无运动分量 */
    if (fabsf(v) < 0.1f && fabsf(w) < 0.1f){
        s_straight_lock_active = false;
        (void)Chassis_Brake();
        return;
    }

    /* 情况 2: 纯直行 (前进或后退，无转向分量) */
    if (fabsf(v) > 0.1f && fabsf(w) < 0.1f){
        uint32_t now_ms = BSP_Time_GetMs();
        float base = v * speed;

        /* 若距离上一次转弯动作结束尚不足 250ms (底盘仍处于转弯角动量释放与滑动过渡期)：
         * 保持纯对称开环驱动，不强行施加直行差速纠偏，避免与转弯残余惯性对抗发生摆头 */
        if ((now_ms - s_last_turn_ms) < RC_TURN_SETTLE_MS){
            s_straight_lock_active = false;
            AppRemoteCar_Drive(base, base);
            return;
        }

        /* 起步静摩擦力前馈 (Stiction Breakaway Boost)：
         * 短按起步最初 45ms 内，若设定速度极低 (如 5%)，临时提升至 18% 以克服齿轮箱静摩擦，
         * 确保左右两电机在按键按下的瞬间同时破阻起动 */
        if ((now_ms - s_straight_start_ms) < 45U && fabsf(base) < 18.0f){
            base = (v > 0.0f) ? 18.0f : -18.0f;
        }

        /* 暂停转弯 250ms 之后底盘完全平稳，以此刻稳定的实际航向角作为直行闭环校准起点！ */
        if (!s_straight_lock_active){
            WIT_IMU_DATA imu = {0};
            (void)WitGetData(&imu);
            s_straight_target_yaw = imu.attitude_deg.yaw;
            s_straight_lock_active = true;
            PID_Reset(&s_straight_sync_pid);
            PID_Reset(&s_speed_sync_pid);

            AppRemoteCar_Drive(base, base);
        }
        return;
    }

    /* 带有转向分量时：刷新转弯时间戳，完全释放直行锁并重置 PID，保证手动转弯手感纯净自由 */
    s_last_turn_ms = BSP_Time_GetMs();
    s_straight_lock_active = false;
    PID_Reset(&s_straight_sync_pid);
    PID_Reset(&s_speed_sync_pid);

    /* 情况 3: 原地旋转 (无直行分量，只有左转或右转)
     * 旋转速度严格与当前设定速度成比例 (0.70x 系数)，保证低速轻柔可控、高速迅速，不再急转暴冲 */
    if (fabsf(v) < 0.1f && fabsf(w) > 0.1f){
        float spin_speed = speed * 0.70f;
        if (spin_speed < RC_DUTY_FLOOR_PCT){
            spin_speed = RC_DUTY_FLOOR_PCT;
        }
        float l = (w * RC_STEER_SIGN) * spin_speed;
        float r = (-w * RC_STEER_SIGN) * spin_speed;

        if (l >  100.0f) { l =  100.0f; }
        if (l < -100.0f) { l = -100.0f; }
        if (r >  100.0f) { r =  100.0f; }
        if (r < -100.0f) { r = -100.0f; }

        AppRemoteCar_Drive(l, r);
        return;
    }

    /* 情况 4: 边行进边转弯 (走弧线，如 W+A / W+D / S+A / S+D)
     * 差速比例与当前 SPD 保持严格线性对应，无论 5% 还是 100% 均保持相同优美曲率 */
    {
        float base = v * speed;
        float diff = 0.45f * speed; /* 45% 差速比例，外轮加速、内轮减速 */
        float l = base + (w * RC_STEER_SIGN) * diff;
        float r = base - (w * RC_STEER_SIGN) * diff;

        if (l >  100.0f) { l =  100.0f; }
        if (l < -100.0f) { l = -100.0f; }
        if (r >  100.0f) { r =  100.0f; }
        if (r < -100.0f) { r = -100.0f; }

        AppRemoteCar_Drive(l, r);
    }
}

/* 收到一帧方向状态（来自 !DIR 或单个 WASD 字符）：更新状态机并立即输出。 */
static void AppRemoteCar_ApplyDir(uint8_t mask){
    const char *act;

    mask &= RC_DIR_MASK_ALL;

    /* 前后 / 左右同时按下 -> 该轴按 0 处理，避免"互相抵消却还标着 FWD" */
    if ((mask & RC_DIR_FWD) && (mask & RC_DIR_REV)){
        mask &= (uint8_t)~(RC_DIR_FWD | RC_DIR_REV);
    }
    if ((mask & RC_DIR_LEFT) && (mask & RC_DIR_RIGHT)){
        mask &= (uint8_t)~(RC_DIR_LEFT | RC_DIR_RIGHT);
    }

    s_last_dir_mask = mask;

    if (mask == 0U){
        s_straight_lock_active = false;
        /* 松手：立刻刹车，但**保持 RUN 状态**（绿灯不灭），
         * 下次按键不用再按一遍 S1。
         * 这一条是"按一下马上松也能立刻停"的关键：不等 400ms 失联保护。 */
        if (s_car_state != 2U){
            s_car_state = 1U;
        }
        s_motion_active = false;
        strncpy((char*)s_last_action, (s_car_state == 2U) ? "TRACK" : "RUN", sizeof(s_last_action));
        (void)Chassis_Brake();
        return;
    }

    /* 手动方向指令触发：自动由循线模式接管为手动 RUN 模式 */
    s_car_state = 1U;
    s_reset_action_ms = 0U;
    bool was_stopped = !s_motion_active;
    s_motion_active = true;
    s_last_motion_ms = BSP_Time_GetMs();

    /* 从停止状态启动纯直行点动：记录起动时刻，赋能 45ms 静摩擦破除脉冲 */
    if (was_stopped && (mask & (RC_DIR_FWD | RC_DIR_REV)) && !(mask & (RC_DIR_LEFT | RC_DIR_RIGHT))){
        s_straight_start_ms = s_last_motion_ms;
    }

    if (mask & (RC_DIR_LEFT | RC_DIR_RIGHT)){
        s_last_turn_ms = BSP_Time_GetMs();
        s_straight_lock_active = false;
        PID_Reset(&s_straight_sync_pid);
    }

    if (mask & RC_DIR_FWD){
        act = (mask & RC_DIR_LEFT) ? "FWD+L" : ((mask & RC_DIR_RIGHT) ? "FWD+R" : "FWD");
    } else if (mask & RC_DIR_REV){
        act = (mask & RC_DIR_LEFT) ? "REV+L" : ((mask & RC_DIR_RIGHT) ? "REV+R" : "REV");
    } else {
        act = (mask & RC_DIR_LEFT) ? "LEFT" : "RIGHT";
    }
    strncpy((char*)s_last_action, act, sizeof(s_last_action));

    AppRemoteCar_MixAndDrive(mask);
}

static void AppRemoteCar_SetSpeedPercent(uint8_t pct){
    if (pct < 5U){
        pct = 5U;
    }
    if (pct > 100U){
        pct = 100U;
    }
    s_speed_pct = pct;

    /* 实时更新循线基础占空比：
     * 0% 对应 0.0f, 50% 对应 22.5f, 100% 对应 45.0f */
    LineTracking_SetBaseDuty((float)pct * 0.45f);

    /* 若小车当前正在运动，按"最近一帧方向掩码"重算一次输出，立即生效。
     *
     * 两个条件缺一不可：
     *   1) s_motion_active —— 只有方向键按下、且还没超时才允许驱动。
     *      旧实现只判断 s_last_action == "RUN" 就前进，于是"旋钮转一下"
     *      就等于发了一条前进指令，车会自己窜出去。
     *   2) 方向指令还没过期 —— 否则停车后转旋钮会重新把车推起来。
     *
     * 这里刻意**不刷新** s_last_motion_ms：转旋钮不算运动意图，
     * 不能让失联保护被调速包无限延长。 */
    if (s_car_state == 1U && s_motion_active &&
        (BSP_Time_GetMs() - s_last_motion_ms) < RC_MOTION_TIMEOUT_MS){
        AppRemoteCar_MixAndDrive(s_last_dir_mask);
    }
}

static void AppRemoteCar_ExecuteCommand(char cmd){
    s_last_cmd_ms = BSP_Time_GetMs();

    switch (cmd){
        case 'w':
        case 'W':
            AppRemoteCar_ApplyDir(RC_DIR_FWD);
            break;

        case 's':
        case 'S':
            AppRemoteCar_ApplyDir(RC_DIR_REV);
            break;

        case 'a':
        case 'A':
            AppRemoteCar_ApplyDir(RC_DIR_LEFT);
            break;

        case 'd':
        case 'D':
            AppRemoteCar_ApplyDir(RC_DIR_RIGHT);
            break;

        case ' ':
        case 'x':
        case 'X':
            s_car_state = 0U;
            s_reset_action_ms = 0U;
            s_motion_active = false;
            s_last_dir_mask = 0U;
            strncpy((char*)s_last_action, "STOP", sizeof(s_last_action));
            (void)Chassis_Brake();
            break;

        case '1': /* S1 键功能：启停切换 */
            s_reset_action_ms = 0U;
            s_last_dir_mask = 0U;
            if (s_car_state == 1U || s_car_state == 2U){
                s_car_state = 0U;
                s_motion_active = false;
                strncpy((char*)s_last_action, "STOP", sizeof(s_last_action));
                (void)Chassis_Brake();
            } else {
                /* RUN 只是"使能/解除使能"，**本身不产生任何运动**。
                 *
                 * 旧实现在这里调用了 AppRemoteCar_Drive(35%, 35%)，
                 * 于是"按一下 S1 进 RUN"= 立刻以 35% 占空比前进，
                 * 没连上位机也会自己往前跑（心跳又不断刷新总看门狗，
                 * 所以车会一直跑下去）。这正是"按键变 RUN 就自动前进"的原因，
                 * 与引脚配置无关。
                 *
                 * 正确语义：进 RUN 后电机保持静止（主动刹车，避免滑行），
                 * 等 WASD / 方向按钮给出方向后才出力。 */
                s_car_state = 1U;
                s_motion_active = false;
                strncpy((char*)s_last_action, "RUN", sizeof(s_last_action));
                (void)Chassis_Brake();
            }
            break;

        case '2': { /* 清零键功能：保持当前模式不变，仅清零里程与航向角 */
            Chassis_ResetDistance();
            WIT_IMU_DATA imu_now = {0};
            (void)WitGetData(&imu_now);
            s_yaw_zero_offset = imu_now.attitude_deg.yaw;
            float g_mag = sqrtf(imu_now.acc_g.x * imu_now.acc_g.x +
                                imu_now.acc_g.y * imu_now.acc_g.y +
                                imu_now.acc_g.z * imu_now.acc_g.z);
            if (g_mag >= 0.7f && g_mag <= 1.3f) {
                s_gravity_ref = g_mag;
            }

            /* 记录先前动作状态；ACT 为 RESET 保持 1 秒后自动恢复 */
            if (strcmp((const char*)s_last_action, "RESET") != 0){
                strncpy((char*)s_prev_action, (const char*)s_last_action, sizeof(s_prev_action));
            }
            strncpy((char*)s_last_action, "RESET", sizeof(s_last_action));
            s_reset_action_ms = BSP_Time_GetMs();
            break;
        }

        case 't':
        case 'T': /* 切换至循线模式 (TRACK) */
            s_car_state = 2U;
            s_motion_active = true;
            s_reset_action_ms = 0U;
            LineTracking_Reset();
            strncpy((char*)s_last_action, "TRACK", sizeof(s_last_action));
            break;

        case 'r':
        case 'R': /* 直接切入 RUN 运行状态 */
        case 'm':
        case 'M': /* 切换至手动遥控模式 (MANUAL/RUN) */
            s_car_state = 1U;
            s_motion_active = false;
            s_reset_action_ms = 0U;
            strncpy((char*)s_last_action, "RUN", sizeof(s_last_action));
            (void)Chassis_Brake();
            break;

        case '3': /* 红绿灯切换 */
            s_traffic_light = (s_traffic_light == 0U) ? 1U : 0U;
            break;

        case '+':
        case '=':
            if (s_speed_pct <= 95U){
                AppRemoteCar_SetSpeedPercent(s_speed_pct + 5U);
            } else {
                AppRemoteCar_SetSpeedPercent(100U);
            }
            break;

        case '-':
        case '_':
            if (s_speed_pct >= 5U){
                AppRemoteCar_SetSpeedPercent(s_speed_pct - 5U);
            } else {
                AppRemoteCar_SetSpeedPercent(0U);
            }
            break;

        case '0':
            AppRemoteCar_SetSpeedPercent(100U);
            break;

        default:
            if (cmd >= '4' && cmd <= '9'){
                AppRemoteCar_SetSpeedPercent((uint8_t)(cmd - '0') * 10U);
            }
            break;
    }
}

static void AppRemoteCar_ParsePacketCommand(const char *pkt, uint8_t src){
    (void)src;
    /* 识别 !CAL:xxx# 巡线模块校准协议 */
    if (strncmp(pkt, "!CAL:", 5) == 0) {
        char resp[128];
        s_last_cmd_ms = BSP_Time_GetMs();

        if (strncmp(pkt + 5, "START", 5) == 0 || strncmp(pkt + 5, "ENTER", 5) == 0) {
            s_calib_state = CALIB_STATE_WAIT_WHITE;
            s_car_state = 0U;
            s_motion_active = false;
            strncpy((char*)s_last_action, "CALIB", sizeof(s_last_action));
            (void)Chassis_Brake();
            AppRemoteCar_Beep(60);
            AppRemoteCar_SendPacket("!CAL:WAIT_WHITE#");
        } else if (strncmp(pkt + 5, "W", 1) == 0) {
            uint8_t w[GRAYSCALE_SENSOR_CHANNEL_COUNT];
            if (GrayscaleSensor_CalibrateSampleWhite(w)) {
                AppRemoteCar_Beep(60);
                snprintf(resp, sizeof(resp), "!CAL:W_OK,%u,%u,%u,%u,%u,%u,%u,%u#",
                         w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
                AppRemoteCar_SendPacket(resp);
                s_calib_state = CALIB_STATE_WAIT_BLACK;
            } else {
                AppRemoteCar_Beep(200);
                AppRemoteCar_SendPacket("!CAL:ERR_I2C#");
            }
        } else if (strncmp(pkt + 5, "B", 1) == 0) {
            uint8_t b[GRAYSCALE_SENSOR_CHANNEL_COUNT];
            uint8_t tw[GRAYSCALE_SENSOR_CHANNEL_COUNT];
            uint8_t tb[GRAYSCALE_SENSOR_CHANNEL_COUNT];
            if (GrayscaleSensor_CalibrateSampleBlack(b) &&
                GrayscaleSensor_ApplyCalibration(tw, tb)) {
                AppRemoteCar_Beep(60);
                for (volatile uint32_t d = 0; d < 200000; d++) { __NOP(); }
                AppRemoteCar_Beep(60);
                snprintf(resp, sizeof(resp), "!CAL:DONE,%u,%u,%u,%u,%u,%u,%u,%u#",
                         tw[0], tw[1], tw[2], tw[3], tw[4], tw[5], tw[6], tw[7]);
                AppRemoteCar_SendPacket(resp);
                s_calib_state = CALIB_STATE_DONE;
                s_calib_done_ms = BSP_Time_GetMs();
            } else {
                AppRemoteCar_Beep(200);
                AppRemoteCar_SendPacket("!CAL:ERR_I2C#");
            }
        } else if (strncmp(pkt + 5, "REQ", 3) == 0 || strncmp(pkt + 5, "INFO", 4) == 0) {
            uint8_t w[GRAYSCALE_SENSOR_CHANNEL_COUNT], b[GRAYSCALE_SENSOR_CHANNEL_COUNT];
            uint8_t tw[GRAYSCALE_SENSOR_CHANNEL_COUNT], tb[GRAYSCALE_SENSOR_CHANNEL_COUNT];
            GrayscaleSensor_GetCalibrationData(w, b, tw, tb);
            snprintf(resp, sizeof(resp), "!CAL:INFO,%u,%u,%u,%u,%u,%u,%u,%u#",
                     tw[0], tw[1], tw[2], tw[3], tw[4], tw[5], tw[6], tw[7]);
            AppRemoteCar_SendPacket(resp);
        } else if (strncmp(pkt + 5, "EXIT", 4) == 0 || strncmp(pkt + 5, "CANCEL", 6) == 0) {
            s_calib_state = CALIB_STATE_IDLE;
            strncpy((char*)s_last_action, "STOP", sizeof(s_last_action));
            AppRemoteCar_SendPacket("!CAL:EXIT#");
        }
        return;
    }

    /* 心跳包识别：任一在线的对端都能压住看门狗 */
    if (strncmp(pkt, "!HB", 3) == 0 || strncmp(pkt, "!PING", 5) == 0){
        s_last_cmd_ms = BSP_Time_GetMs();
        return;
    }

    /* !DIR:<mask># —— 上位机打包的"当前按住了哪些方向键"
     *   bit0=W 前进  bit1=S 后退  bit2=A 左  bit3=D 右
     * 这是"能走斜线 + 松手立刻停"的关键：一帧里就带齐了所有方向，
     * 不用像单字符那样靠先后顺序覆盖。 */
    if (strncmp(pkt, "!DIR:", 5) == 0){
        int m = 0;
        if (sscanf(pkt + 5, "%d", &m) == 1){
            s_last_cmd_ms = BSP_Time_GetMs();
            AppRemoteCar_ApplyDir((uint8_t)m);
        }
        return;
    }

    /* !D:<L>,<R># —— 直接下发左右轮占空比（千分比 -1000~+1000）
     *
     * 为什么需要它：4 档方向掩码（!DIR）只能表达"前进/后退/左/右"八种组合，
     * 上位机做路径跟随时无法下发"左轮 38%、右轮 46%"这种连续纠偏量，
     * 只能靠 bang-bang 打锯齿。这条指令把底盘变成一个"差速轮速接口"。
     *
     * 刻意走 Chassis_SetDuty 而**不**经过 MixAndDrive 的 30% 死区补偿：
     * 那个补偿会把 35/28 抬成 35/30，微调量全被吃掉，跟随就抖成锯齿。
     * 上位机侧保证非零占空比不低于 33%，堵转由它负责。 */
    if (strncmp(pkt, "!D:", 3) == 0){
        int l = 0;
        int r = 0;
        if (sscanf(pkt + 3, "%d,%d", &l, &r) == 2){
            float dl = (float)l / 10.0f;      /* 千分比 -> 百分比 */
            float dr = (float)r / 10.0f;

            if (dl >  100.0f) { dl =  100.0f; }
            if (dl < -100.0f) { dl = -100.0f; }
            if (dr >  100.0f) { dr =  100.0f; }
            if (dr < -100.0f) { dr = -100.0f; }

            s_last_cmd_ms = BSP_Time_GetMs();
            s_car_state = 1U;
            s_reset_action_ms = 0U;
            /* 收到上位机连续差速指令 !D 时，清空直行闭环标志与方向掩码，
             * 防止与 50Hz 直行同步控速(AppRemoteCar_StraightSyncStep)抢夺控制权 */
            s_last_dir_mask = 0U;
            s_straight_lock_active = false;

            if (dl == 0.0f && dr == 0.0f){
                /* 显式停车：解除"允许出力"，让后续失联保护不被误触发 */
                s_motion_active = false;
                strncpy((char*)s_last_action, "RUN", sizeof(s_last_action));
                (void)Chassis_Brake();
            } else {
                s_motion_active = true;
                s_last_motion_ms = s_last_cmd_ms;
                strncpy((char*)s_last_action, "AUTO", sizeof(s_last_action));
                (void)Chassis_SetDuty(dl, dr);
            }
        }
        return;
    }

    /* 识别 !SET:SPD=xx# 格式 */
    if (strncmp(pkt, "!SET:SPD=", 9) == 0){
        int val = 0;
        if (sscanf(pkt + 9, "%d", &val) == 1){
            if (val < 0) val = 0;
            if (val > 100) val = 100;
            AppRemoteCar_SetSpeedPercent((uint8_t)val);
            s_last_cmd_ms = BSP_Time_GetMs();
        }
        return;
    }

    /* 识别 !MODE:xxx# 模式切换格式 (TRACK / MANUAL / STOP) */
    if (strncmp(pkt, "!MODE:", 6) == 0){
        s_last_cmd_ms = BSP_Time_GetMs();
        if (strncmp(pkt + 6, "TRACK", 5) == 0 || strncmp(pkt + 6, "2", 1) == 0){
            s_car_state = 2U;
            s_motion_active = true;
            s_reset_action_ms = 0U;
            LineTracking_Reset();
            strncpy((char*)s_last_action, "TRACK", sizeof(s_last_action));
        } else if (strncmp(pkt + 6, "STOP", 4) == 0 || strncmp(pkt + 6, "0", 1) == 0){
            s_car_state = 0U;
            s_motion_active = false;
            s_reset_action_ms = 0U;
            strncpy((char*)s_last_action, "STOP", sizeof(s_last_action));
            (void)Chassis_Brake();
        } else {
            s_car_state = 1U;
            s_motion_active = false;
            s_reset_action_ms = 0U;
            strncpy((char*)s_last_action, "RUN", sizeof(s_last_action));
            (void)Chassis_Brake();
        }
        return;
    }

    /* 识别 !SET:KP=xx# 和 !SET:KD=xx# 巡线 PID 参数在线调节 */
    if (strncmp(pkt, "!SET:KP=", 8) == 0){
        float kp = 1.2f;
        if (sscanf(pkt + 8, "%f", &kp) == 1){
            LineTracking_SetPid(kp, 0.0f, 1.8f);
            s_last_cmd_ms = BSP_Time_GetMs();
        }
        return;
    }
    if (strncmp(pkt, "!SET:KD=", 8) == 0){
        float kd = 1.8f;
        if (sscanf(pkt + 8, "%f", &kd) == 1){
            LineTracking_SetPid(1.2f, 0.0f, kd);
            s_last_cmd_ms = BSP_Time_GetMs();
        }
        return;
    }
}

static void AppRemoteCar_Dispatch(uint8_t src, uint8_t byte){
    RC_RX_PARSER *p = &s_rx_parser[src];

    /* 上位机在线判定：只统计真正来自无线模块的字节 */
    if (src == RC_SRC_PC){
        s_last_pc_ms = BSP_Time_GetMs();
    }
    /* MSP430 终端在线判定：任何来自 UART0 的字节（含 2Hz 心跳 !HB#） */
    if (src == RC_SRC_MSP430){
        s_last_msp_ms = BSP_Time_GetMs();
    }

    /* 封包指令处理 '!...#' */
    if (byte == '!'){
        p->idx = 0U;
        p->buf[p->idx++] = (char)byte;
        return;
    }

    if (p->idx > 0U){
        if (p->idx < (RC_CMD_BUFFER_LEN - 1U)){
            p->buf[p->idx++] = (char)byte;
        }
        if (byte == '#'){
            p->buf[p->idx] = '\0';
            AppRemoteCar_ParsePacketCommand(p->buf, src);
            p->idx = 0U;
        } else if (p->idx >= (RC_CMD_BUFFER_LEN - 1U)){
            p->idx = 0U;        /* 超长且无 '#': 直接丢弃，避免永久卡在封包态 */
        }
        return;
    }

    /* 心跳包快速响应 */
    if (byte == 'h' || byte == 'H'){
        s_last_cmd_ms = BSP_Time_GetMs();
        return;
    }

    /* 单字符按键指令 */
    AppRemoteCar_ExecuteCommand((char)byte);
}

void AppRemoteCar_ProcessRxByte(uint8_t byte){
    AppRemoteCar_Dispatch(RC_SRC_CANMV, byte);
}

void AppRemoteCar_ProcessPcByte(uint8_t byte){
    AppRemoteCar_Dispatch(RC_SRC_PC, byte);
}

void AppRemoteCar_ProcessMsp430Byte(uint8_t byte){
    AppRemoteCar_Dispatch(RC_SRC_MSP430, byte);
}

bool AppRemoteCar_IsMsp430Alive(uint32_t timeout_ms){
    return (s_last_msp_ms > 0U) &&
           ((uint32_t)(BSP_Time_GetMs() - s_last_msp_ms) <= timeout_ms);
}

static void AppRemoteCar_SendTelemetryFrame(void){
    uint8_t frame[20];
    WIT_IMU_DATA imu = {0};
    (void)WitGetData(&imu);

    CHASSIS_DUTY duty = Chassis_GetDuty();
    /* V：直接取霍尔编码器实测轮速（m/s -> mm/s）。
     * 旧实现里 s_car_state==0 时会把速度强制写 0，导致"车没启动/被推着走"
     * 时仪表永远显示 0.00，看起来像没接编码器。现在只做 ±10mm/s 的静止噪声死区。 */
    float raw_speed = Chassis_GetSpeed();
    int16_t spd_mm = (int16_t)(raw_speed * 1000.0f);
    if (spd_mm > -10 && spd_mm < 10){
        spd_mm = 0;
    }

    int32_t dist_mm = (int32_t)(Chassis_GetDistance() * 1000.0f);

    /* 计算基于清零点的相对航向角 (0.1° 分辨率, 范围 -180.0° ~ +180.0°) */
    float rel_yaw = imu.attitude_deg.yaw - s_yaw_zero_offset;
    while (rel_yaw > 180.0f)  rel_yaw -= 360.0f;
    while (rel_yaw < -180.0f) rel_yaw += 360.0f;
    int16_t yaw_01 = (int16_t)(rel_yaw * 10.0f);

    /* A：直接取 JY61P 的 X 轴（车头方向）加速度，单位 0.01 g。
     * 旧实现用的是 |a| - |g| 的"模长差"，那对水平加速是二次小量：
     * 实际 0.20 g 的前向加速只会算成 sqrt(1+0.04)-1 ≈ 0.02 g，
     * 看起来永远停在 0.00，很容易误判成"没取到 IMU"。 */
    float ax = imu.acc_g.x;
    int32_t acc_raw = (int32_t)(ax * 100.0f);

    /* ±0.01g 死区：JY61P 静止时 X 轴会在 ±0.01g 之间来回跳，
     * 屏幕上就是 +0.01 / -0.01 反复闪。把它压成 0.00，
     * 静止时读数完全稳定，动起来才出数。 */
    if (acc_raw >= -1 && acc_raw <= 1) {
        acc_raw = 0;
    }

    if (acc_raw > 300)  { acc_raw = 300; }      /* 限幅 ±3.00 g，防溢出 */
    if (acc_raw < -300) { acc_raw = -300; }
    int16_t acc_001g = (int16_t)acc_raw;

    /* 重力基准仍持续跟踪，供 S2 键清零 / 其它逻辑使用 */
    {
        float a_tot = sqrtf(imu.acc_g.x * imu.acc_g.x +
                            imu.acc_g.y * imu.acc_g.y +
                            imu.acc_g.z * imu.acc_g.z);
        if (s_car_state == 0U && a_tot >= 0.7f && a_tot <= 1.3f) {
            s_gravity_ref = 0.95f * s_gravity_ref + 0.05f * a_tot;
        }
    }

    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = s_speed_pct;         /* 实时下传当前生效限速百分比 */
    /* [3] 低 7 位 = 运行状态(0=STOP,1=RUN,3=BRAKE)，最高位 = 上位机(无线)在线。
     * MSP430 的 LINK/DISC 就用这一位，从而和天猛星的三色灯永远同步：
     * 红灯亮 <=> 这一位为 0 <=> 屏幕显示 DISC；
     * 黄/绿灯亮 <=> 这一位为 1 <=> 屏幕显示 LINK。 */
    frame[3] = (uint8_t)((s_car_state & 0x7FU) |
                         (AppRemoteCar_IsPcOnline() ? RC_FLAG_PC_ONLINE : 0U));
    frame[4] = s_traffic_light;     /* 交通灯状态: 0=RED, 1=GREEN */
    frame[5] = (uint8_t)((uint16_t)spd_mm >> 8);
    frame[6] = (uint8_t)((uint16_t)spd_mm & 0xFFU);
    frame[7] = (uint8_t)((uint32_t)dist_mm >> 24);
    frame[8] = (uint8_t)((uint32_t)dist_mm >> 16);
    frame[9] = (uint8_t)((uint32_t)dist_mm >> 8);
    frame[10] = (uint8_t)((uint32_t)dist_mm & 0xFFU);
    frame[11] = (uint8_t)((uint16_t)yaw_01 >> 8);
    frame[12] = (uint8_t)((uint16_t)yaw_01 & 0xFFU);
    frame[13] = (uint8_t)((uint16_t)acc_001g >> 8);    /* 总加速度高字节 */
    frame[14] = (uint8_t)((uint16_t)acc_001g & 0xFFU); /* 总加速度低字节 */
    frame[15] = (uint8_t)(int8_t)duty.left_percent;
    frame[16] = (uint8_t)(int8_t)duty.right_percent;

    uint8_t sum = 0U;
    for (uint8_t i = 2U; i <= 16U; i++){
        sum += frame[i];
    }
    frame[17] = sum;
    frame[18] = 0x55U;
    frame[19] = 0xAAU;

    /* 20Hz 定长遥测帧，同时下发两路：
     *   UART1 (PA8/PA9) -> 无线透传模块 -> PC 上位机
     *   UART0 (PB0/PB1) -> MSP430 显示终端
     * 视觉 K230(UART2/PA24/PB15) 不再复送：本车按实际接线未使用该路，
     * 每帧 20 字节的阻塞发送只是白占 1.7ms。 */
    for (uint8_t i = 0U; i < 20U; i++){
        DL_UART_Main_transmitDataBlocking(Wireless_INST, frame[i]);
        DL_UART_Main_transmitDataBlocking(Debug_INST, frame[i]);
    }
}

void App_RunRemoteCarTask(void){
    uint32_t last_telemetry_ms = 0U;
    uint32_t last_ui_ms = 0U;

    /* 注册原始串口接收回调，截获来自 MSP430 / PC 的控制数据 */
    CanMvUart_SetRawByteCallback(AppRemoteCar_ProcessRxByte);

    /* 初始化 JY61P 硬件 I2C0 接口与中断 */
    JY61P_I2C_Init();

    /* 初始化感为 I2C 灰度传感器与巡线闭环控制器 */
    GrayscaleSensor_Init();
    LineTracking_Init(NULL);
    LineTracking_SetBaseDuty((float)s_speed_pct * 0.45f);

    /* 初始化直行两轮同步 (航向锁) 与编码器闭环控制器 */
    AppRemoteCar_InitControllers();

    s_last_cmd_ms = BSP_Time_GetMs();
    s_last_motion_ms = s_last_cmd_ms;
    s_motion_active = false;
    Key_ClearAllEvents();

    while (1){
        /* 轮询推进 JY61P I2C0 状态机 (非阻塞) */
        JY61P_I2C_Poll();

        /* 天猛星板载 DOWN 按键 (PB25) 巡线校准状态机推进 */
        if (Key_Down_IsShortPress()) {
            AppRemoteCar_HandleDownKey();
        }

        uint32_t now = BSP_Time_GetMs();

        /* ACT 为 RESET 保持一秒后自动回到 RESET 之前的动作状态 */
        if (s_reset_action_ms > 0U && (now - s_reset_action_ms) >= 1000U){
            s_reset_action_ms = 0U;
            strncpy((char*)s_last_action, (const char*)s_prev_action, sizeof(s_last_action));
        }

        /* 三色指示灯（同一时刻只亮一盏）：
         *   无线未连接(链路故障)   -> 红灯
         *   已连接 + STOP/BRAKE    -> 黄灯（待机，推方向键就能走）
         *   已连接 + RUN/TRACK     -> 绿灯
         *
         * 颜色语义按"红=故障、黄=待机、绿=运行"来排，与 MSP430 板上那颗
         * 红灯（链路断开时亮）保持一致；早先的版本是"断链黄、STOP 红"，
         * 两块板对同一个故障给出相反颜色，现场看着就像反了。
         *
         * 连接判据只看 UART1(无线) 收到的字节，MSP430 的 2Hz 心跳不参与，
         * 否则拔掉无线模块后仍然显示"已连接"。 */
        bool is_connected = AppRemoteCar_IsPcOnline();
        bool is_running   = (s_car_state == 1U || s_car_state == 2U);

        bool led_red    = false;
        bool led_yellow = false;
        bool led_green  = false;

        if (!is_connected){
            led_red = true;                 /* 断链 = 故障 */
        } else if (is_running){
            led_green = true;               /* 已连接 + RUN */
        } else {
            led_yellow = true;              /* 已连接 + STOP = 待机 */
        }

        /* 先全灭再点亮，保证任何时刻只有一盏亮（三色灯直连 GPIO，没有互锁） */
        DL_GPIO_clearPins(LED_R_PORT, LED_R_PIN);
        DL_GPIO_clearPins(LED_Y_PORT, LED_Y_PIN);
        DL_GPIO_clearPins(LED_G_PORT, LED_G_PIN);

        if (led_red){
            DL_GPIO_setPins(LED_R_PORT, LED_R_PIN);
        } else if (led_yellow){
            DL_GPIO_setPins(LED_Y_PORT, LED_Y_PIN);
        } else if (led_green){
            DL_GPIO_setPins(LED_G_PORT, LED_G_PIN);
        }

        /* 安全看门狗：如果处于行进状态且超过设定时间未收到任何指令，自动刹车 */
        if (s_car_state == 1U && (now - s_last_cmd_ms) > RC_WATCHDOG_TIMEOUT_MS){
            s_car_state = 0U;
            s_motion_active = false;
            s_last_dir_mask = 0U;
            strncpy((char*)s_last_action, "TIMEOUT", sizeof(s_last_action));
            (void)Chassis_Brake();
        }

        /* 方向指令死手开关：RUN 状态下如果 400ms 内没有新的方向指令，
         * 立刻停转（状态仍是 RUN，可以随时再按方向键）。
         *
         * 这一条同时兜住三种情况：
         *   1) 上位机松开 WASD 后不再补发 -> 车立刻停，不会因为 1.2s 总看门狗
         *      而多冲出去一段；
         *   2) 无线丢包 / USB 拔掉 -> 方向指令停更，车自己停住；
         *   3) 任何残留的方向指令（例如上一次实验留下的 s_last_action）
         *      在超时后都不再产生输出，RUN 之后默认就是静止。 */
        if (s_motion_active &&
            (now - s_last_motion_ms) >= RC_MOTION_TIMEOUT_MS){
            s_motion_active = false;
            s_straight_lock_active = false;
            (void)Chassis_Brake();
            if (s_car_state == 1U){
                /* s_last_action / s_last_dir_mask 也会被串口中断里的指令改写，
                 * 这里短暂关中断，避免两份 strncpy 交错成半截字符串。 */
                __disable_irq();
                s_last_dir_mask = 0U;
                strncpy((char*)s_last_action, "RUN", sizeof(s_last_action));
                __enable_irq();
            }
        }

        /* 遥控直行状态 50Hz 闭环同步控速 (双轮速度闭环同步保持直线)
         * 核心：转弯期间绝不干涉；暂停转弯 250ms 后以稳定新航向为基准自动无缝闭环 */
        static uint32_t s_last_sync_loop_ms = 0U;
        if (s_car_state == 1U && s_motion_active && (now - s_last_sync_loop_ms) >= 20U){
            float dt_s = (s_last_sync_loop_ms == 0U) ? 0.02f : ((float)(now - s_last_sync_loop_ms) / 1000.0f);
            s_last_sync_loop_ms = now;

            /* 判断当前指令是否为纯直行 (无左右转向分量) */
            bool is_pure_straight = ((s_last_dir_mask & (RC_DIR_FWD | RC_DIR_REV)) != 0U) &&
                                    ((s_last_dir_mask & (RC_DIR_LEFT | RC_DIR_RIGHT)) == 0U);

            if (is_pure_straight){
                if (!s_straight_lock_active){
                    /* 检查转弯结束过渡期是否已满 250ms */
                    if ((now - s_last_turn_ms) >= RC_TURN_SETTLE_MS){
                        /* 暂停转弯之后的一时刻：精准校准并锁定当前稳定航向角为直行闭环起点！ */
                        WIT_IMU_DATA imu = {0};
                        (void)WitGetData(&imu);
                        s_straight_target_yaw = imu.attitude_deg.yaw;
                        s_straight_lock_active = true;
                        PID_Reset(&s_straight_sync_pid);
                        PID_Reset(&s_speed_sync_pid);
                    }
                } else {
                    /* 航向已平稳锁定，执行 50Hz 直行差速纠偏 */
                    AppRemoteCar_StraightSyncStep(dt_s);
                }
            } else {
                /* 只要存在转向分量，直行闭环完全避让退出，并持续刷新转弯时刻 */
                s_last_turn_ms = now;
                s_straight_lock_active = false;
            }
        }

        /* 循线模式（TRACK, s_car_state == 2）50Hz 闭环控制 */
        static uint32_t s_last_track_ms = 0U;
        if (s_car_state == 2U && (now - s_last_track_ms) >= 20U){
            float dt_s = (s_last_track_ms == 0U) ? 0.02f : ((float)(now - s_last_track_ms) / 1000.0f);
            s_last_track_ms = now;

            (void)LineFollow_Update();
            LINE_FOLLOW_SENSOR_STATE sensor;
            (void)LineFollow_GetSensor(&sensor);

            LINE_TRACKING_OUTPUT track_out;
            (void)LineTracking_Compute(&sensor, dt_s, &track_out);

            if (track_out.line_lost){
                /* 丢线安全保护：停车刹车，防止失控冲出场地 */
                strncpy((char*)s_last_action, "LOST", sizeof(s_last_action));
                (void)Chassis_Brake();
            } else {
                strncpy((char*)s_last_action, "TRK", sizeof(s_last_action));
                (void)Chassis_SetDuty(track_out.left_duty, track_out.right_duty);
            }
        }

        /* 无论车动不动，均以严格 20Hz 周期性回传遥测数据包给 MSP430 / PC */
        if ((now - last_telemetry_ms) >= RC_TELEMETRY_PERIOD_MS){
            last_telemetry_ms = now;
            AppRemoteCar_SendTelemetryFrame();
        }

        /* 10Hz OLED 陀螺仪指南针仪表盘刷新 (或校准向导界面) */
        if ((now - last_ui_ms) >= RC_UI_REFRESH_PERIOD_MS){
            last_ui_ms = now;

            if (s_calib_state != CALIB_STATE_IDLE) {
                if (s_calib_state == CALIB_STATE_WAIT_WHITE) {
                    OLED_DrawCalibrationScreen(1U, "Press [DOWN] on White");
                } else if (s_calib_state == CALIB_STATE_WAIT_BLACK) {
                    OLED_DrawCalibrationScreen(2U, "Press [DOWN] on Black");
                } else if (s_calib_state == CALIB_STATE_DONE) {
                    OLED_DrawCalibrationScreen(3U, "Calibrated & Saved!");
                    if ((now - s_calib_done_ms) >= 1500U) {
                        s_calib_state = CALIB_STATE_IDLE;
                        strncpy((char*)s_last_action, "STOP", sizeof(s_last_action));
                    }
                }
            } else {
            WIT_IMU_DATA imu = {0};
            (void)WitGetData(&imu);

            float disp_yaw = imu.attitude_deg.yaw - s_yaw_zero_offset;
            while (disp_yaw > 180.0f)  disp_yaw -= 360.0f;
            while (disp_yaw < -180.0f) disp_yaw += 360.0f;

            float disp_spd = Chassis_GetSpeed();
            if (s_car_state == 0U || (disp_spd > -0.015f && disp_spd < 0.015f)){
                disp_spd = 0.0f;
            }

            /* 陀螺仪健康度直接判读（不再靠"数值看起来不动"猜）：
             *   NOACK = 从机不应答  -> 查地址/供电/接线/模块是否在 I2C 模式
             *   BUS   = 总线被拉死  -> 查上拉电阻/线长/从机是否半字节死锁
             *   WAIT  = 上电后尚未读成功
             *   STALE = 曾经读到但已超过 500ms 未更新 */
            const char *act_text;
            switch (JY61P_I2C_GetHealth()){
                case JY61P_HEALTH_NACK:    act_text = "NOACK"; break;
                case JY61P_HEALTH_TIMEOUT: act_text = "BUS";   break;
                case JY61P_HEALTH_OK:
                    act_text = JY61P_I2C_IsDataFresh(500U)
                                 ? (const char *)s_last_action
                                 : "STALE";
                    break;
                default:                   act_text = "WAIT";  break;
            }

            OLED_DrawCompassDashboard(disp_yaw,
                                      s_speed_pct,
                                      act_text,
                                      disp_spd,
                                      Chassis_GetDistance(),
                                      s_traffic_light);
            }
        }

        BSP_DelayMs(5U);
    }

    /* 退出任务清理 */
    CanMvUart_SetRawByteCallback(NULL);
    (void)Chassis_Brake();
}
