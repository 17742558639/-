#ifndef _MOVE_H
#define _MOVE_H

#include "main.h"
#include "headfile.h"

#define Move_CMD 4      // 电机状态位，数据包4个字节
#define Move_Wtite_CMD 3      // 电机状态位，数据包3个字节

/* ============================================================
   定位闭环参数 —— 把"厘米"换算成"电机脉冲数"
   ============================================================
   换算链条：目标距离(cm) → 轮子转多少圈 → 电机转多少脉冲

   轮子转一圈 = π × 直径 = 3.14159 × 8 ≈ 25.13cm
   电机转一圈 = 360° ÷ 步距角(1.8°) × 细分倍数 = 200 × 细分

   举例：走 50cm，细分=8
     圈数 = 50 ÷ 25.13 ≈ 2 圈
     脉冲 = 2 × (200×8) = 2 × 1600 = 3200 脉冲
   ============================================================ */
#define WHEEL_DIAMETER_CM       8.0f        /* 麦克纳姆轮直径(cm)，拿尺子量的 */
#define STEP_ANGLE              1.8f        /* 步进电机步距角(度)，电机铭牌参数 */
#define FULL_STEPS_PER_REV      (uint16_t)(360.0f / STEP_ANGLE)  /* 一整圈需要 200 个整步 */

#define MICROSTEP               16          /* 细分倍数：16细分 */
#define PULSES_PER_REV          (uint16_t)(FULL_STEPS_PER_REV * MICROSTEP)  /* 每圈脉冲数 = 200×16 = 3200 */

#define WHEEL_CIRCUMFERENCE     (3.14159f * WHEEL_DIAMETER_CM)   /* 轮子周长 ≈ 25.13cm */
/* 厘米 → 脉冲数：先用小数乘除，最后转成整数 */
#define CM_TO_PULSES(cm)        ((uint32_t)((float)(cm) / WHEEL_CIRCUMFERENCE * (float)PULSES_PER_REV))

/* Absolute vehicle yaw for the rear-mounted arm/camera. */
#define MATERIAL_STATION_YAW   0
#define ROUGH_STATION_YAW    180
#define STORAGE_STATION_YAW   90
#define QR_SCAN_YAW          -90

extern void Move_Init(void);
extern void Move_Idle_Func(void);
extern uint32_t my_abs(int data);
void RunMove(uint8_t dir,uint16_t distance,uint8_t isrunmap);
void RunMoveWithIMU(uint8_t dir, uint16_t distance);
extern uint8_t Rotation(short angle_desired);
uint8_t TurnIMU(short delta_deg);
extern void PidMove(int x,int y);
extern void Stop(void);
uint8_t RunMap(void);
void CoarseProcesing(void);
void CoarseNavigateTo(uint8_t target);
void Procesing(void);
uint32_t my_abs(int data);

typedef struct
{
	uint8_t  dir[4];	// �ĸ������˶��ķ���
	uint8_t  vel[4]; 		// ���е�����ٶ�
	uint8_t  acc;		// �Ӽ���
}xMOVE,*pxMOVE;

typedef struct {
	uint8_t  acc;		// �Ӽ���
	int x;    			// X��
	int y;   			// Y��
    int omega; 			// ��ת
}xMOVETAR,*pxMOVETAR;


/*��ͼʱ(x,y,dis)��ز���*/ 

#define 	RUNMOVE_VEL			88

enum Dir
{
	LEFT,
	RIGHT,
	FRONT,
	BACK
};

#endif
