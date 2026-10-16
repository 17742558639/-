#include "pid.h"

PidObject imu_pid;
PidObject x_pid;
PidObject y_pid;

//����ʽpid���ٶȻ���
void PidInit_z(PidObject_Z *pid, float KP, float KI, int Max, int Min)
{
    pid->error = 0;
    pid->preverror = 0;
    pid->integ = 0;
    pid->deriv = 0;
    pid->kp = KP;
    pid->ki = KI;
    pid->MaxValue = Max;
    pid->MinValue = Min;
    pid->out = 0;
    pid->out_last = 0;
    pid->sum = 0;
}
float pidout_z(PidObject_Z *pid, float desired, float measure)
{
    pid->desired = desired;
    pid->error = pid->desired - measure;
    pid->deriv = pid->error - pid->preverror;
    pid->preverror = pid->error;
    pid->out = pid->out_last + ((float)(pid->kp * pid->deriv + pid->ki * pid->error));
    pid->out_last = pid->out;

    // ����޷�
    if (pid->out > pid->MaxValue)
        pid->out = pid->MaxValue;
    if (pid->out < pid->MinValue)
        pid->out = pid->MinValue;

    return pid->out;
}

//λ��ʽpid��ת�򻻣�
void PidInit(PidObject *pid, float desired, float KP,float KI, float KD, int Max, int Min)
{
    pid->error = 0;
    pid->preverror = 0;
    pid->integ = 0;
    pid->deriv = 0;
    pid->desired = desired;
    pid->kp = KP;
    pid->ki = KI;
    pid->kd = KD;
    pid->out = 0;
    pid->MaxValue = Max;
    pid->MinValue = Min;
}
float pidout(PidObject *pid, float measure)
{
    pid->error = (pid->desired - measure);
    pid->integ += pid->error;
    pid->deriv =  pid->error - pid->preverror; 
    pid->preverror = pid->error;
    
    pid->out = (pid->kp * pid->error + pid->kd * pid->deriv);
       
    // ����޷�
    if (pid->out > pid->MaxValue)
        pid->out = pid->MaxValue;
    if (pid->out < pid->MinValue)
        pid->out = pid->MinValue;

    return pid->out;
}

float pidout_i(PidObject *pid, float measure)
{
    pid->error = (pid->desired - measure);

    /* 死区：误差太小就当0，避免来回抖（冠军 ±0.5°） */
    if (pid->error > -IMU_PID_DEADZONE && pid->error < IMU_PID_DEADZONE)
        pid->error = 0;

    pid->integ += pid->error;
    pid->deriv =  pid->error - pid->preverror;
    pid->preverror = pid->error;

    pid->out = (pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv);

    // 输出限幅
    if (pid->out > pid->MaxValue)
        pid->out = pid->MaxValue;
    if (pid->out < pid->MinValue)
        pid->out = pid->MinValue;

    return pid->out;
}

void PID_Init(void)
{
    PidInit(
      &imu_pid,
      0,
      IMU_PID_P,
      IMU_PID_I,
      IMU_PID_D,
      IMU_PID_MAX,
      IMU_PID_MIN
    );

    PidInit(
      &x_pid,
      0,
      X_PID_P,
      0,
      0,
      X_PID_MAX,
      X_PID_MIN
    );

    PidInit(
      &y_pid,
      0,
      Y_PID_P,
      0,
      0,
      Y_PID_MAX,
      Y_PID_MIN
    );

}

void IMUPidClear(void)
{
    imu_pid.error = 0;
    imu_pid.preverror = 0;
    imu_pid.integ = 0;
    imu_pid.deriv = 0;
    imu_pid.out = 0;
}

