#ifndef _IMU_H
#define _IMU_H

#include "main.h"

#define  IMU_CMD  11

extern __IO short yaw;
extern __IO short yaw_desired;

void Imu_Init(void);	
void Imu_Idle_Func(void);	
uint8_t Imu_Wait_Ready(uint32_t timeout_ms);

#endif
