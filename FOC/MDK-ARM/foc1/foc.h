#ifndef __FOC_OPENLOOP_H
#define __FOC_OPENLOOP_H

#include "main.h"
#include "tim.h"
#include "math.h"
#include "stm32f10x.h"                  // Device header


// ????
#define MOTOR_POLE_PAIRS      7
#define PWM_FREQUENCY         20000    // 20kHz PWM??
#define PWM_PERIOD            1000     // PWM?????
#define VOLTAGE_LIMIT         0.95f    // ???? (0-1)

// FOC??
typedef enum {
    FOC_STOP = 0,
    FOC_ALIGN,      // ????
    FOC_OPENLOOP,   // ????
    FOC_CLOSEDLOOP  // ???? (??)
} FOC_State;

// FOC?????
typedef struct {
    FOC_State state;
    
    // ????
    float target_speed_elec;    // ????? (rad/s)
    float current_speed_elec;   // ????? (rad/s)
    float electrical_angle;     // ??? (rad)
    
    // ?????
    float id_ref;               // d?????
    float iq_ref;               // q?????
    
    // ????
    float v_alpha;
    float v_beta;
    float v_d;
    float v_q;
    
    // PWM??
    float duty_u;
    float duty_v;
    float duty_w;
    
    // ????
    float voltage_limit;
    float alignment_angle;
    uint32_t alignment_time;
    uint32_t start_time;
    
    uint8_t is_initialized;
} FOC_OpenLoop_HandleTypeDef;

// ????
void FOC_OpenLoop_Init(FOC_OpenLoop_HandleTypeDef *hfoc);
void FOC_OpenLoop_Start(FOC_OpenLoop_HandleTypeDef *hfoc, float speed_rpm);
void FOC_OpenLoop_Stop(FOC_OpenLoop_HandleTypeDef *hfoc);
void FOC_OpenLoop_SetSpeed(FOC_OpenLoop_HandleTypeDef *hfoc, float speed_rpm);
void FOC_OpenLoop_SetCurrent(FOC_OpenLoop_HandleTypeDef *hfoc, float iq);
void FOC_OpenLoop_Update(FOC_OpenLoop_HandleTypeDef *hfoc, float dt);
void FOC_OpenLoop_Align(FOC_OpenLoop_HandleTypeDef *hfoc);
void FOC_OpenLoop_GeneratePWM(FOC_OpenLoop_HandleTypeDef *hfoc);

// ????
void Clarke_Transform(float ia, float ib, float ic, float *i_alpha, float *i_beta);
void Park_Transform(float i_alpha, float i_beta, float angle, float *i_d, float *i_q);
void Inverse_Park_Transform(float v_d, float v_q, float angle, float *v_alpha, float *v_beta);
void SVM_Transform(float v_alpha, float v_beta, float *duty_u, float *duty_v, float *duty_w);

#endif