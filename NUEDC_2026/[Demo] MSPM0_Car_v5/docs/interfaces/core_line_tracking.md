# core/line_tracking 接口说明

## 模块职责

`core/line_tracking` 提供巡线控制算法。

该模块从 `middleware/line_follow` 获取八路灰度状态，使用 `core/pid` 计算转向修正，通过 `core/kinematics` 做差速混控，最后调用 `middleware/chassis` 输出左右轮占空比。

灰度传感器读取属于 BSP，巡线状态维护属于 middleware，具体题目流程仍属于 app。

## 数据结构

```c
typedef struct {
    float base_duty;
    float sensor_position_scale;
    float output_limit;
    float differential_limit;
    PID_CONFIG pid_config;
} LINE_TRACKING_CONFIG;
```

- `base_duty`：基础前进占空比百分比。
- `sensor_position_scale`：灰度传感器位置到误差的缩放系数。
- `output_limit`：左右轮最终占空比限幅。
- `differential_limit`：循迹时左右轮占空比差值限幅，`<= 0` 表示不启用。
- `pid_config`：巡线 PID 参数。

```c
typedef struct {
    float error;
    float correction;
    float left_duty;
    float right_duty;
    uint8_t active_count;
    bool line_lost;
} LINE_TRACKING_OUTPUT;
```

- `error`：当前巡线偏差。
- `correction`：PID 输出的转向修正。
- `left_duty` / `right_duty`：最终左右轮占空比。
- `active_count`：检测到线的灰度通道数量。
- `line_lost`：是否未检测到线。

## 默认参数

```c
#define LINE_TRACKING_DEFAULT_BASE_DUTY 28.0f
#define LINE_TRACKING_DEFAULT_OUTPUT_LIMIT 100.0f
#define LINE_TRACKING_DEFAULT_CORRECTION_LIMIT 35.0f
#define LINE_TRACKING_DEFAULT_DIFFERENTIAL_LIMIT 50.0f
#define LINE_TRACKING_DEFAULT_POSITION_SCALE 1.0f
#define LINE_TRACKING_ACTIVE_SENSOR_MASK 0xFFU
```

默认 PID 为位置式 PID（纯 PD，现场一般不需要积分）：

```text
kp = 4.2
ki = 0.0
kd = 0.5
integral_limit = 50.0
output_limit = 35.0
```

八路探头横向位置为 `-3.5 ~ +3.5`，加权平均后得到偏差；中心通道对齐时 `|error| < 0.20` 直接归零（死区），
再做一阶低通 `0.70*old + 0.30*new`，抑制探头离散跳变导致的微分冲击。

`LINE_TRACKING_ACTIVE_SENSOR_MASK` 用于选择参与巡线控制的灰度通道。当前值 `0xFF` 启用 0 到 7 的全部 8 路灰度传感器。

## 在线整定（上位机 → 小车）

参数可以在网页「循线 PID 调节」面板里直接改，走无线串口文本包，不需要重新烧录固件：

| 方向 | 报文 | 说明 |
| --- | --- | --- |
| 上位机 → 小车 | `!SET:PID=4.2,0,0.5#` | 三项一起下发（推荐） |
| 上位机 → 小车 | `!SET:KP=4.2#` / `!SET:KI=0#` / `!SET:KD=0.5#` | 单项下发，兼容旧上位机；未指定的项保持原值 |
| 上位机 → 小车 | `!GET:PID#` | 回读当前参数 |
| 小车 → 上位机 | `!PID=420,0,50#` | 回读/设置确认帧，数值为 `×100` 的整数 |

回读帧用整数是为了绕开 MicroLIB 不带浮点 `printf` 的限制。固件在 `app/app_remote_car_task.c`
里解析并调用 `LineTracking_SetPid()`，取值范围在 app 层被钳到 `kp ∈ [0,20]`、`ki ∈ [0,5]`、`kd ∈ [0,10]`。

调参顺序：先把 `Ki`、`Kd` 置零，只调 `Kp` 到能跟过所有弯、直道刚要开始摆头；再加 `Kd`（0.3~0.8）压住摆动；
`Ki` 保持 0，只有长期单侧蹭线才加 0.01~0.05。**换速度后要重看一遍**：基础占空比由上位机左侧限速滑块决定，
同一组增益在不同速度下阻尼比不同。

## 接口

### `void LineTracking_Init(const LINE_TRACKING_CONFIG *config)`

初始化巡线控制器。传入 `NULL` 时使用默认配置。

### `void LineTracking_Reset(void)`

清空 PID 状态和最近一次输出。

### `void LineTracking_SetBaseDuty(float duty)` / `float LineTracking_GetBaseDuty(void)`

动态设置/读取巡线基础占空比（0~100%），对应上位机的「当前动力输出限速」。

### `void LineTracking_SetPid(float kp, float ki, float kd)` / `void LineTracking_GetPid(float *kp, float *ki, float *kd)`

在线设置/回读巡线 PID 参数。设置时会用新的 `pid_config` 重新初始化 PID 控制器（同时清零积分项），
所以改参数不会带入上一组参数残留的积分状态。回读接口的输出指针允许传 `NULL`。

### `BSP_STATUS LineTracking_Update(float dt_s)`

完整巡线闭环入口：

1. 调用 `LineFollow_Update()` 更新灰度状态。
2. 调用 `LineTracking_Compute()` 计算左右轮占空比，只统计 `LINE_TRACKING_ACTIVE_SENSOR_MASK` 启用的通道。
3. 若未丢线，则调用 `Chassis_SetDuty()` 输出到底盘。

循迹输出会先把 PID 转向修正量限制在 `differential_limit * 0.5` 内（默认 ±25%），
再执行最终输出限幅。当前默认差值上限为 `50%`。该限制只影响 `LineTracking_Update()` 路径，不影响 `core/motion` 的直行、倒车或原地转向动作。

如果当前未检测到线，函数返回 `BSP_STATUS_NOT_READY`，不主动输出底盘占空比。题目流程可根据自身策略决定刹车、继续搜索或切换状态。

### `BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor, float dt_s, LINE_TRACKING_OUTPUT *out)`

纯计算接口，不读取传感器，也不输出到底盘。传入空指针时返回 `BSP_STATUS_NULL`。

### `LINE_TRACKING_OUTPUT LineTracking_GetOutput(void)`

返回最近一次巡线计算结果。

## 边界说明

- 空线、半线、十字、边线计数等流程判断不在本模块公开，后续应在 `middleware/line_follow` 或 app 任务流程中按实际需要设计。
- 直角弯识别和转弯动作编排不在本模块中实现；当前 E1 在 `app/app_e_task` 中通过任务状态机组合 `core/motion` 原语完成。
- 旧 `sInedge` 和 `UpdateSInedge()` 不迁入本模块。
- 旧 `Motion_Car_Control()` 的职责已经拆分为 `Kinematics_DifferentialMix()` 和 `Chassis_SetDuty()`。
