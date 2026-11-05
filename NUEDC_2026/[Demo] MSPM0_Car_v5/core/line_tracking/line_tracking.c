#include "line_tracking.h"

#include "chassis.h"
#include "kinematics/kinematics.h"
#include <math.h>
#include <stddef.h>

static const float s_sensor_position[LINE_FOLLOW_SENSOR_COUNT] = {
    -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f,
};

static bool LineTracking_IsSensorEnabled(uint8_t index){
    return ((LINE_TRACKING_ACTIVE_SENSOR_MASK & (1U << index)) != 0U);
}

static LINE_TRACKING_CONFIG s_line_tracking_config;
static LINE_TRACKING_OUTPUT s_line_tracking_output;
static PID_CONTROLLER s_line_tracking_pid;
static bool s_line_tracking_initialized = false;
static float s_filtered_error = 0.0f;

static LINE_TRACKING_CONFIG LineTracking_DefaultConfig(void){
    LINE_TRACKING_CONFIG config = {
        .base_duty = LINE_TRACKING_DEFAULT_BASE_DUTY,
        .sensor_position_scale = LINE_TRACKING_DEFAULT_POSITION_SCALE,
        .output_limit = LINE_TRACKING_DEFAULT_OUTPUT_LIMIT,
        .differential_limit = LINE_TRACKING_DEFAULT_DIFFERENTIAL_LIMIT,
        .pid_config = {
            .kp = 4.2f,
            .ki = 0.0f,
            .kd = 0.5f,
            .integral_limit = 50.0f,
            .output_limit = LINE_TRACKING_DEFAULT_CORRECTION_LIMIT,
            .mode = PID_MODE_POSITION,
        },
    };

    return config;
}

static void LineTracking_EnsureInitialized(void){
    if (!s_line_tracking_initialized){
        LineTracking_Init(NULL);
    }
}

void LineTracking_Init(const LINE_TRACKING_CONFIG *config){
    if (config == NULL){
        s_line_tracking_config = LineTracking_DefaultConfig();
    } else{
        s_line_tracking_config = *config;
    }

    PID_Init(&s_line_tracking_pid, &s_line_tracking_config.pid_config);
    LineTracking_Reset();
    s_line_tracking_initialized = true;
}

void LineTracking_Reset(void){
    PID_Reset(&s_line_tracking_pid);
    s_filtered_error = 0.0f;

    s_line_tracking_output.error = 0.0f;
    s_line_tracking_output.correction = 0.0f;
    s_line_tracking_output.left_duty = 0.0f;
    s_line_tracking_output.right_duty = 0.0f;
    s_line_tracking_output.active_count = 0U;
    s_line_tracking_output.line_lost = true;
}

void LineTracking_SetBaseDuty(float duty){
    LineTracking_EnsureInitialized();
    if (duty < 0.0f)   { duty = 0.0f; }
    if (duty > 100.0f) { duty = 100.0f; }
    s_line_tracking_config.base_duty = duty;
}

float LineTracking_GetBaseDuty(void){
    LineTracking_EnsureInitialized();
    return s_line_tracking_config.base_duty;
}

void LineTracking_SetPid(float kp, float ki, float kd){
    LineTracking_EnsureInitialized();
    s_line_tracking_config.pid_config.kp = kp;
    s_line_tracking_config.pid_config.ki = ki;
    s_line_tracking_config.pid_config.kd = kd;
    PID_Init(&s_line_tracking_pid, &s_line_tracking_config.pid_config);
}

void LineTracking_GetPid(float *kp, float *ki, float *kd){
    LineTracking_EnsureInitialized();
    if (kp != NULL){ *kp = s_line_tracking_config.pid_config.kp; }
    if (ki != NULL){ *ki = s_line_tracking_config.pid_config.ki; }
    if (kd != NULL){ *kd = s_line_tracking_config.pid_config.kd; }
}

BSP_STATUS LineTracking_Update(float dt_s){
    LineTracking_EnsureInitialized();

    BSP_STATUS status = LineFollow_Update();
    if (status != BSP_STATUS_OK){
        return status;
    }

    LINE_FOLLOW_SENSOR_STATE sensor;
    status = LineFollow_GetSensor(&sensor);
    if (status != BSP_STATUS_OK){
        return status;
    }

    status = LineTracking_Compute(&sensor, dt_s, &s_line_tracking_output);
    if (status != BSP_STATUS_OK){
        return status;
    }

    if (s_line_tracking_output.line_lost){
        return BSP_STATUS_NOT_READY;
    }

    return Chassis_SetDuty(s_line_tracking_output.left_duty,
                           s_line_tracking_output.right_duty);
}

BSP_STATUS LineTracking_Compute(const LINE_FOLLOW_SENSOR_STATE *sensor,
                                float dt_s,
                                LINE_TRACKING_OUTPUT *out){
    if ((sensor == NULL) || (out == NULL)){
        return BSP_STATUS_NULL;
    }

    LineTracking_EnsureInitialized();

    float position_sum = 0.0f;
    uint8_t active_count = 0U;

    /*
     * 8 路传感器横向位置加权平均。
     * 巡线语义：sensor->value[i] == 0 表示探测到黑线。
     */
    for (uint8_t i = 0; i < LINE_FOLLOW_SENSOR_COUNT; i++){
        if (LineTracking_IsSensorEnabled(i) && sensor->value[i] == 0U){
            position_sum += s_sensor_position[i];
            active_count++;
        }
    }

    out->active_count = active_count;
    out->line_lost = (active_count == 0U);

    if (out->line_lost){
        out->error = 0.0f;
        out->correction = 0.0f;
        out->left_duty = 0.0f;
        out->right_duty = 0.0f;
        return BSP_STATUS_OK;
    }

    /* 横向偏差：负数偏左，正数偏右 */
    float average_position = position_sum / (float)active_count;

    /* 死区抑制（Deadband）：中心通道 3与4 (对应 -0.5 与 +0.5) 居中对齐时偏差 < 0.20 直接归零，彻底根除直行持续蛇形摆头 */
    float target_error = 0.0f;
    if (fabsf(average_position) >= 0.20f) {
        target_error = average_position * s_line_tracking_config.sensor_position_scale;
    }

    /* 一阶低通平滑滤波：消除 8 路离散探头状态切换时的阶跃冲击，防止微分项产生极端极限环震荡 */
    s_filtered_error = 0.70f * s_filtered_error + 0.30f * target_error;
    out->error = s_filtered_error;

    out->correction = PID_Update(&s_line_tracking_pid, out->error, 0.0f, dt_s);

    if (s_line_tracking_config.differential_limit > 0.0f){
        float correction_limit = s_line_tracking_config.differential_limit * 0.5f;
        out->correction = Kinematics_Clamp(out->correction,
                                           -correction_limit,
                                           correction_limit);
    }

    /* 左右轮差速混合输出 */
    KINEMATICS_DIFFERENTIAL_OUTPUT duty =
        Kinematics_DifferentialMix(s_line_tracking_config.base_duty,
                                   out->correction,
                                   s_line_tracking_config.output_limit);

    out->left_duty = duty.left;
    out->right_duty = duty.right;

    return BSP_STATUS_OK;
}

LINE_TRACKING_OUTPUT LineTracking_GetOutput(void){
    return s_line_tracking_output;
}
