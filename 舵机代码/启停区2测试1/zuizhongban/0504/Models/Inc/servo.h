#ifndef _SERVO_H
#define _SERVO_H

#include "main.h"
#include "headfile.h"

void Servo_SetAngle(uint32_t which,float angle);
void Servo_Init(void); 
void Catch_Init(uint32_t which); // 兼容旧任务入口，初始化实测观察姿态
void Servo_SetAngle_Slow(uint32_t which, float angle, float step, int step_delay);
void Servo_Close_To_Plate(void);
void Servo_Watch_To_Move(void);

/* 仅舵机姿态动作：暂不接入车辆/摄像头流程。 */
void Servo_To_Scan_A(void);       // A低位摄像头扫描姿态
void Servo_Scan_A_To_Place_B(void); // A姿态到B低位放置姿态
void Servo_Release_At_Place(void); // B姿态打开夹爪释放物料

void Catch(uint32_t which);
void Place(uint32_t which);

void Servo_Move_To_Plate(uint16_t which_plate); 
void Servo_Plate_To_Move(uint16_t which_plate);

extern uint8_t plate_state[3];


#define     JOINT1_FRONT            90  // 实测：正对站点/正前方；舵机1顺时针读数减小
#define     JOINT1_PLATE1           292 // 实测：第一盘子抓取位置
#define     JOINT1_PLATE2           325 // 实测：第二盘子抓取位置
#define     JOINT1_PLATE3           360 // 实测：第三盘子抓取位置

#define     JOINT2_WATCH            220 // 实测：观察/正前方；读数增加表示靠近地面
#define     JOINT2_CATCH            360 // 实测：正前方站点实际抓取，已到舵机2机械极限
#define     JOINT2_MOVE             260 // 实测：搬运/A扫描姿态的J2
#define     JOINT2_PLATE_GRAB       280 // 三个盘子共用：抓取姿态J2


#define     JOINT3_WATCH            38  // 实测：观察/正前方；读数增加表示远离地面
#define     JOINT3_CATCH            150 // 实测：正前方站点实际抓取姿态
#define     JOINT3_MOVE             110 // 实测：搬运姿态的J3
#define     JOINT3_PLATE_GRAB       100 // 三个盘子共用：抓取姿态J3


#define     JOINT4_CATCH            0  // 实测：闭合、夹紧物料；读数减小表示闭合
#define     JOINT4_PLATE            60 // 实测：观察/正前方姿态的打开值
#define     JOINT4_OPEN             70 // 实测：二维码扫描姿态的打开值
#define     JOINT4_PLATE_GRAB       0  // 三个盘子共用：抓取时保持闭合

/* === 按实测表独立记录的姿态，避免把不同流程误用成同一个宏 === */
#define     JOINT4_WATCH            60 // 观察、正前方：半开
#define     JOINT2_QR1              220 // 二维码候选姿态A（二选一，只扫描一次）：J2=220,J3=100,J4=70
#define     JOINT3_QR1              100
#define     JOINT2_QR2              90  // 二维码候选姿态B（二选一，只扫描一次）：J2=90,J3=0,J4=70
#define     JOINT3_QR2              0
#define     JOINT2_SCAN_A           260 // A低位摄像头扫描：J2=260,J3=60,J4=0
#define     JOINT3_SCAN_A           60
#define     JOINT2_PLACE_B          360 // B低位放置：J2=360，已到舵机2机械极限
#define     JOINT3_PLACE_B          150
#define     JOINT4_PLACE_B          0   // 到达B姿态后保持夹紧，释放时再打开

#define     PLATE_NUM               3

#endif
