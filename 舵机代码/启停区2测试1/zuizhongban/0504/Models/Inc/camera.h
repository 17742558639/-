#ifndef _CAMERA_H
#define _CAMERA_H

#include "main.h"
#include "headfile.h"

extern uint8_t str[4];
extern __IO uint8_t CameraReady;	

extern void Camera_Init(void);
extern void Camera_Idle_Func(void);
extern void SetCameraTask(uint8_t which);
extern void WaitPlace(uint8_t which);

#define     CAMERA_CMD          8

#define     PLATE_DESIRED_X     130
#define     PLATE_DESIRED_Y     136

#define     ITEM_DESIRED_X      144
#define     ITEM_DESIRED_Y      72

// 和摄像头处理任务一一对应
enum CamTask
{
    NOTHING,    // 占位
    CAM_ITEM_A_1,
    CAM_ITEM_B_1,
    CAM_ITEM_C_1,
    CAM_ITEM_D_1,
    CAM_PLACE_A,
    CAM_PLACE_B,
    CAM_PLACE_C,
    CAM_PLACE_D,
    CAM_PLACE_a,
    CAM_PLACE_b,
    CAM_PLACE_c,
    CAM_PLACE_d,
    CAM_PLACE_e,
    CAM_PLACE_f,
    CAM_PLACE_abc,
    CAM_PLACE_def,
    CAM_PLACE_HOME,
    CAM_YOLO
};


typedef struct {
    short class_id;
    short num;
} YoloResult;

typedef struct {
    YoloResult candidate;      // 当前候选结果
    int consecutive_count;     // 连续相同次数
    YoloResult stable_result;  // 最终稳定结果
    unsigned long start_time;  // 开始识别时间
    int is_stable;             // 是否已稳定
} StableDetector;

void reset_detector(StableDetector *det);
extern StableDetector detector;
void WaitYolo(void);

#endif
