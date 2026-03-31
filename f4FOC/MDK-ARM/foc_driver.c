/*
 * @File: foc_driver.c
 * @Description: SimpleFOC Architecture Implementation
 */

#include "foc_driver.h"

// AS5600 相关
#define AS5600_I2C_ADDR         0x36
#define AS5600_RAW_HI_REG       0x0C
#define AS5600_I2C_TIMEOUT_MS   10

extern I2C_HandleTypeDef hi2c2;

// --- 基础数学 ---
static float _normalizeAngle(float angle) {
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

// --- 传感器读取 ---
// 返回 0xFFFF 表示错误
uint16_t AS5600_ReadRawAngle(void) {
    uint8_t buf[2];
    if(HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(AS5600_I2C_ADDR << 1), 
                        AS5600_RAW_HI_REG, I2C_MEMADD_SIZE_8BIT, 
                        buf, 2, AS5600_I2C_TIMEOUT_MS) != HAL_OK) return 0xFFFF;
    return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}

void Sensor_Update(FOC_t *hfoc, float dt) {
    uint16_t raw = AS5600_ReadRawAngle();
    if (raw == 0xFFFF) return; // 容错

    float val = (float)(raw & 0x0FFF) * 2.0f * PI / 4096.0f;
    float d_angle = val - hfoc->encoder.angle_raw;
    
    // 圈数处理
    if (abs(d_angle) > (0.8f * 2 * PI)) {
        hfoc->encoder.full_rotations += (d_angle > 0) ? -1 : 1;
    }
    if (d_angle > PI) d_angle -= 2 * PI;
    else if (d_angle < -PI) d_angle += 2 * PI;

    hfoc->encoder.angle_raw = val;
    hfoc->encoder.angle_consequent += d_angle;
    hfoc->shaft_angle = hfoc->encoder.angle_consequent; // 机械角度

    // 速度计算
    if (dt > 0.0001f) {
        float vel_raw = d_angle / dt;
        // 使用 LPF 滤波后的速度
        hfoc->shaft_velocity = LPF_Operator(&hfoc->lpf_vel, vel_raw, dt);
    }
}

// --- 低通滤波器 (Low Pass Filter) ---
// y(k) = alpha * x(k) + (1-alpha) * y(k-1)
// alpha = dt / (Tf + dt)
void LPF_Init(LowPassFilter_t *lpf, float time_constant) {
    lpf->Tf = time_constant;
    lpf->prev_output = 0.0f;
}

float LPF_Operator(LowPassFilter_t *lpf, float x, float dt) {
    float alpha = dt / (lpf->Tf + dt);
    float y = alpha * x + (1.0f - alpha) * lpf->prev_output;
    lpf->prev_output = y;
    return y;
}

// --- PID 控制器 (带 Ramp) ---
void PID_Init(PID_Controller_t *pid, float P, float I, float D, float ramp, float limit) {
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp; // 0 表示不限制
    pid->limit = limit;
    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
}

float PID_Operator(PID_Controller_t *pid, float error, float dt) {
    if (dt <= 0.0001f) dt = 0.001f;

    // P项
    float proportional = pid->P * error;

    // I项 (Tustin 变换或简单累加)
    float integral = pid->integral_prev + pid->I * error * dt;
    integral = LIMIT(integral, -pid->limit, pid->limit);
    pid->integral_prev = integral;

    // D项 (一般速度环不用D，位置环可能用)
    // float derivative = (error - pid->error_prev) / dt;
    // pid->error_prev = error;

    float output = proportional + integral;
    output = LIMIT(output, -pid->limit, pid->limit);

    // Ramp (电压爬升限制) - 让电机启动更柔和，不突变
    if (pid->output_ramp > 0) {
        float output_rate = (output - pid->output_prev) / dt;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * dt;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * dt;
    }
    
    pid->output_prev = output;
    return output;
}

// --- SVPWM 核心 ---
static void _setPwm(FOC_t *hfoc, float Ua, float Ub, float Uc) {
    // 限制电压
    float limit = hfoc->voltage_limit;
    Ua = LIMIT(Ua, -limit, limit);
    Ub = LIMIT(Ub, -limit, limit);
    Uc = LIMIT(Uc, -limit, limit);

    // 零序注入 (SVPWM)
    float v_mid = 0.5f * (fminf(Ua, fminf(Ub, Uc)) + fmaxf(Ua, fmaxf(Ub, Uc)));
    Ua -= v_mid;
    Ub -= v_mid;
    Uc -= v_mid;

    // 转换占空比
    float dc_a = LIMIT((Ua / hfoc->voltage_ps) + 0.5f, 0.0f, 1.0f);
    float dc_b = LIMIT((Ub / hfoc->voltage_ps) + 0.5f, 0.0f, 1.0f);
    float dc_c = LIMIT((Uc / hfoc->voltage_ps) + 0.5f, 0.0f, 1.0f);

    __HAL_TIM_SetCompare(hfoc->PWMPin.tim1, hfoc->PWMPin.PIN1, (uint32_t)(dc_a * hfoc->PWMPin.tim1->Init.Period));
    __HAL_TIM_SetCompare(hfoc->PWMPin.tim2, hfoc->PWMPin.PIN2, (uint32_t)(dc_b * hfoc->PWMPin.tim2->Init.Period));
    __HAL_TIM_SetCompare(hfoc->PWMPin.tim3, hfoc->PWMPin.PIN3, (uint32_t)(dc_c * hfoc->PWMPin.tim3->Init.Period));
}

void FOC_SetTorque(FOC_t *hfoc, float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el);
    float s = sinf(angle_el);
    float c = cosf(angle_el);

    // Inverse Park
    hfoc->Ualpha = Ud * c - Uq * s;
    hfoc->Ubeta  = Ud * s + Uq * c;

    // Inverse Clarke
    hfoc->Ua = hfoc->Ualpha;
    hfoc->Ub = (-0.5f * hfoc->Ualpha) + (_SQRT3 / 2.0f * hfoc->Ubeta);
    hfoc->Uc = (-0.5f * hfoc->Ualpha) - (_SQRT3 / 2.0f * hfoc->Ubeta);

    _setPwm(hfoc, hfoc->Ua, hfoc->Ub, hfoc->Uc);
}

// --- 初始化与校准 ---
void FOC_Init(FOC_t *hfoc, float v_supply, float v_limit, int dir, int pp) {
    memset(hfoc, 0, sizeof(FOC_t));
    hfoc->voltage_ps = v_supply;
    hfoc->voltage_limit = v_limit;
    hfoc->dir = dir;
    hfoc->pp = pp;
    
    // 默认参数：LPF 时间常数 0.01s (100Hz)
    LPF_Init(&hfoc->lpf_vel, 0.01f);
}

void FOC_PWM_Config(FOC_t *hfoc, TIM_HandleTypeDef *t1, TIM_HandleTypeDef *t2, TIM_HandleTypeDef *t3, uint32_t ch1, uint32_t ch2, uint32_t ch3) {
    hfoc->PWMPin.tim1 = t1;
    hfoc->PWMPin.tim2 = t2;
    hfoc->PWMPin.tim3 = t3;
    hfoc->PWMPin.PIN1 = ch1;
    hfoc->PWMPin.PIN2 = ch2;
    hfoc->PWMPin.PIN3 = ch3;
    HAL_TIM_PWM_Start(t1, ch1);
    HAL_TIM_PWM_Start(t2, ch2);
    HAL_TIM_PWM_Start(t3, ch3);
}

void FOC_Alignment(FOC_t *hfoc) {
    // 强行吸合校准
    FOC_SetTorque(hfoc, 0, 3.0f, 0); // 加到 Ud
    HAL_Delay(1000);
    Sensor_Update(hfoc, 0); // 读一次
    
    // 零点
    hfoc->zero_electric_angle = _normalizeAngle((float)(hfoc->dir * hfoc->pp) * hfoc->encoder.angle_raw);
    FOC_SetTorque(hfoc, 0, 0, 0);
}

// --- 主循环 ---
void FOC_Loop(FOC_t *hfoc, float dt) {
    // 1. 传感器更新 (包含 LPF 滤波)
    Sensor_Update(hfoc, dt);
    
    // 2. 电角度计算
    float el_angle = _normalizeAngle((float)(hfoc->dir * hfoc->pp) * hfoc->encoder.angle_raw - hfoc->zero_electric_angle);
    hfoc->electrical_angle = el_angle;

    float voltage_q = 0, voltage_d = 0;

    switch(hfoc->mode) {
        case FOC_MODE_OPEN_LOOP:
            hfoc->electrical_angle = _normalizeAngle(hfoc->electrical_angle + hfoc->target * dt); 
            voltage_q = 3.0f; // 开环电压
            break;
            
        case FOC_MODE_VELOCITY:
            // 速度环 PID
            voltage_q = PID_Operator(&hfoc->pid_vel, hfoc->target - hfoc->shaft_velocity, dt);
            break;
            
        // ... 其他模式 ...
    }

    FOC_SetTorque(hfoc, voltage_q, voltage_d, hfoc->electrical_angle);
}