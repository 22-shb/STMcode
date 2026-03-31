#include "foc.h"

// ????
FOC_OpenLoop_HandleTypeDef hfoc;

// PWM?????
#define PWM_TIMER &htim1

// ????
#define SQRT3     1.73205080757f
#define SQRT3_2   0.86602540378f
#define TWO_PI    6.28318530718f
#define PI_OVER_3 1.04719755120f

void FOC_OpenLoop_Init(FOC_OpenLoop_HandleTypeDef *hfoc)
{
    hfoc->state = FOC_STOP;
    hfoc->target_speed_elec = 0.0f;
    hfoc->current_speed_elec = 0.0f;
    hfoc->electrical_angle = 0.0f;
    
    hfoc->id_ref = 0.0f;
    hfoc->iq_ref = 0.3f;  // ??q?????
    
    hfoc->v_alpha = 0.0f;
    hfoc->v_beta = 0.0f;
    hfoc->v_d = 0.0f;
    hfoc->v_q = 0.0f;
    
    hfoc->duty_u = 0.0f;
    hfoc->duty_v = 0.0f;
    hfoc->duty_w = 0.0f;
    
    hfoc->voltage_limit = VOLTAGE_LIMIT;
    hfoc->alignment_angle = 0.0f;
    hfoc->alignment_time = 1000; // 1?????
    hfoc->start_time = 0;
    hfoc->is_initialized = 1;
    
    // ????PWM??
    HAL_TIM_PWM_Stop(PWM_TIMER, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(PWM_TIMER, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(PWM_TIMER, TIM_CHANNEL_3);
    
    // ??PWM?0
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_3, 0);
}

void FOC_OpenLoop_Start(FOC_OpenLoop_HandleTypeDef *hfoc, float speed_rpm)
{
    if (hfoc->state == FOC_STOP) {
        hfoc->state = FOC_ALIGN;
        hfoc->target_speed_elec = (speed_rpm * TWO_PI * MOTOR_POLE_PAIRS) / 60.0f;
        hfoc->current_speed_elec = 0.0f;
        hfoc->electrical_angle = 0.0f;
        hfoc->start_time = HAL_GetTick();
        
        // ??PWM??
        HAL_TIM_PWM_Start(PWM_TIMER, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(PWM_TIMER, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(PWM_TIMER, TIM_CHANNEL_3);
        
        // ????
        FOC_OpenLoop_Align(hfoc);
    }
}

void FOC_OpenLoop_Stop(FOC_OpenLoop_HandleTypeDef *hfoc)
{
    hfoc->state = FOC_STOP;
    hfoc->target_speed_elec = 0.0f;
    hfoc->current_speed_elec = 0.0f;
    hfoc->electrical_angle = 0.0f;
    
    // ??PWM??
    HAL_TIM_PWM_Stop(PWM_TIMER, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(PWM_TIMER, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(PWM_TIMER, TIM_CHANNEL_3);
    
    // ??PWM?0
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_3, 0);
}

void FOC_OpenLoop_SetSpeed(FOC_OpenLoop_HandleTypeDef *hfoc, float speed_rpm)
{
    if (hfoc->state != FOC_STOP) {
        hfoc->target_speed_elec = (speed_rpm * TWO_PI * MOTOR_POLE_PAIRS) / 60.0f;
    }
}

void FOC_OpenLoop_SetCurrent(FOC_OpenLoop_HandleTypeDef *hfoc, float iq)
{
    hfoc->iq_ref = iq;
    if (hfoc->iq_ref > 1.0f) hfoc->iq_ref = 1.0f;
    if (hfoc->iq_ref < -1.0f) hfoc->iq_ref = -1.0f;
}

// ?????
void FOC_OpenLoop_Update(FOC_OpenLoop_HandleTypeDef *hfoc, float dt)
{
    if (hfoc->state == FOC_STOP) {
        return;
    }
    
    // ?????
    switch (hfoc->state) {
        case FOC_ALIGN:
            // ???? - ??????
            if (HAL_GetTick() - hfoc->start_time > hfoc->alignment_time) {
                hfoc->state = FOC_OPENLOOP;
                hfoc->current_speed_elec = 0.1f; // ????
            }
            break;
            
        case FOC_OPENLOOP:
            // ???? - ?????????
            if (hfoc->current_speed_elec < hfoc->target_speed_elec) {
                float acceleration = 10.0f; // ???? rad/s²
                hfoc->current_speed_elec += acceleration * dt;
                if (hfoc->current_speed_elec > hfoc->target_speed_elec) {
                    hfoc->current_speed_elec = hfoc->target_speed_elec;
                }
            }
            break;
            
        default:
            break;
    }
    
    // ?????
    hfoc->electrical_angle += hfoc->current_speed_elec * dt;
    
    // ?????0-2p???
    if (hfoc->electrical_angle >= TWO_PI) {
        hfoc->electrical_angle -= TWO_PI;
    }
    if (hfoc->electrical_angle < 0.0f) {
        hfoc->electrical_angle += TWO_PI;
    }
    
    // FOC????
    // 1. ?????? (?????????)
    hfoc->v_d = 0.0f;           // d????0 (????)
    hfoc->v_q = hfoc->iq_ref * hfoc->voltage_limit; // q???????
    
    // 2. ?Park?? (dq -> aß)
    Inverse_Park_Transform(hfoc->v_d, hfoc->v_q, hfoc->electrical_angle, 
                          &hfoc->v_alpha, &hfoc->v_beta);
    
    // 3. SVPWM?? (aß -> UVW)
    SVM_Transform(hfoc->v_alpha, hfoc->v_beta, 
                  &hfoc->duty_u, &hfoc->duty_v, &hfoc->duty_w);
    
    // 4. ??PWM
    FOC_OpenLoop_GeneratePWM(hfoc);
}

// ?????? - ??????????
void FOC_OpenLoop_Align(FOC_OpenLoop_HandleTypeDef *hfoc)
{
    // ?????,??????????????????
    hfoc->v_alpha = 0.5f * hfoc->voltage_limit;
    hfoc->v_beta = 0.0f;
    
    SVM_Transform(hfoc->v_alpha, hfoc->v_beta, 
                  &hfoc->duty_u, &hfoc->duty_v, &hfoc->duty_w);
    
    FOC_OpenLoop_GeneratePWM(hfoc);
}

// ??PWM??
void FOC_OpenLoop_GeneratePWM(FOC_OpenLoop_HandleTypeDef *hfoc)
{
    // ???????PWM???
    uint16_t pwm_u = (uint16_t)(hfoc->duty_u * PWM_PERIOD);
    uint16_t pwm_v = (uint16_t)(hfoc->duty_v * PWM_PERIOD);
    uint16_t pwm_w = (uint16_t)(hfoc->duty_w * PWM_PERIOD);
    
    // ??PWM???????
    if (pwm_u > PWM_PERIOD) pwm_u = PWM_PERIOD;
    if (pwm_v > PWM_PERIOD) pwm_v = PWM_PERIOD;
    if (pwm_w > PWM_PERIOD) pwm_w = PWM_PERIOD;
    if (pwm_u < 0) pwm_u = 0;
    if (pwm_v < 0) pwm_v = 0;
    if (pwm_w < 0) pwm_w = 0;
    
    // ??PWM????
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_1, pwm_u);
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_2, pwm_v);
    __HAL_TIM_SET_COMPARE(PWM_TIMER, TIM_CHANNEL_3, pwm_w);
}

// Clarke?? (???? -> ????)
void Clarke_Transform(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
    *i_alpha = ia;
    *i_beta = (ia + 2.0f * ib) / SQRT3;
}

// Park?? (???? -> ????)
void Park_Transform(float i_alpha, float i_beta, float angle, float *i_d, float *i_q)
{
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
    
    *i_d = i_alpha * cos_angle + i_beta * sin_angle;
    *i_q = -i_alpha * sin_angle + i_beta * cos_angle;
}

// ?Park?? (???? -> ????)
void Inverse_Park_Transform(float v_d, float v_q, float angle, float *v_alpha, float *v_beta)
{
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
    
    *v_alpha = v_d * cos_angle - v_q * sin_angle;
    *v_beta = v_d * sin_angle + v_q * cos_angle;
}

// ??????SVPWM (???? -> ?????)
void SVM_Transform(float v_alpha, float v_beta, float *duty_u, float *duty_v, float *duty_w)
{
    // ??????
    float u = v_alpha;
    float v = (-v_alpha + SQRT3 * v_beta) / 2.0f;
    float w = (-v_alpha - SQRT3 * v_beta) / 2.0f;
    
    // ????????????PWM
    float offset = (fmaxf(fmaxf(u, v), w) + fminf(fminf(u, v), w)) / 2.0f;
    
    // ?????????0-1??
    *duty_u = (u - offset + 1.0f) / 2.0f;
    *duty_v = (v - offset + 1.0f) / 2.0f;
    *duty_w = (w - offset + 1.0f) / 2.0f;
    
    // ???0-1???
    if (*duty_u > 1.0f) *duty_u = 1.0f;
    if (*duty_v > 1.0f) *duty_v = 1.0f;
    if (*duty_w > 1.0f) *duty_w = 1.0f;
    if (*duty_u < 0.0f) *duty_u = 0.0f;
    if (*duty_v < 0.0f) *duty_v = 0.0f;
    if (*duty_w < 0.0f) *duty_w = 0.0f;
}

// ??????
FOC_State FOC_OpenLoop_GetState(FOC_OpenLoop_HandleTypeDef *hfoc)
{
    return hfoc->state;
}

// ???????
float FOC_OpenLoop_GetElectricalAngle(FOC_OpenLoop_HandleTypeDef *hfoc)
{
    return hfoc->electrical_angle;
}

// ???????? (RPM)
float FOC_OpenLoop_GetSpeedRPM(FOC_OpenLoop_HandleTypeDef *hfoc)
{
    return (hfoc->current_speed_elec * 60.0f) / (TWO_PI * MOTOR_POLE_PAIRS);
}

// ??????
void FOC_OpenLoop_SetVoltageLimit(FOC_OpenLoop_HandleTypeDef *hfoc, float limit)
{
    if (limit > 1.0f) limit = 1.0f;
    if (limit < 0.0f) limit = 0.0f;
    hfoc->voltage_limit = limit;
}