#ifndef __PID_H
#define __PID_H

#include "main.h"                  // Device header
#include "headfile.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float desired;
    float error;
    float preverror;
    float integ;
    float deriv;
    float MaxValue;
    float MinValue;
    float out;
}PidObject;


typedef struct
{
    float kp;
    float ki;
    float desired;
    float error;
    float preverror;
    float integ;
    float deriv;
    float MaxValue;
    float MinValue;
    float out;
    float out_last;
    float sum;
}PidObject_Z;

extern PidObject imu_pid;
extern PidObject x_pid;
extern PidObject y_pid;


#define     IMU_PID_P   -4      // Kp: 偏1°用4的速度修正（冠军4.0，这里负号是方向约定）
#define     IMU_PID_I   0       // Ki: 不用积分，纯PD控制
#define     IMU_PID_D   -3      // Kd: 微分阻尼，防震荡（冠军3.0）
#define     IMU_PID_DEADZONE  0.5f  // 死区±0.5°，偏不到半度就当没偏，省得来回抖
#define     IMU_PID_MAX   200     // 输出限幅（冠军±200）
#define     IMU_PID_MIN   -200

#define     X_PID_P     -0.5    
#define     X_PID_MAX   100
#define     X_PID_MIN   -100  

#define     Y_PID_P     0.5   
#define     Y_PID_MAX   100
#define     Y_PID_MIN   -100


void PidInit(PidObject *pid, float desired, float KP,float KI, float KD, int Max, int Min);
float pidout(PidObject *pid, float measure);

void PidInit_z(PidObject_Z *pid, float KP, float KI, int Max, int Min);
float pidout_z(PidObject_Z *pid, float desired, float measure);

void PID_Init(void);

float pidout_i(PidObject *pid, float measure);
void IMUPidClear(void);

#endif /* PID */
