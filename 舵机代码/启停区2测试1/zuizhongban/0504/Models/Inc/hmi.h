#ifndef _HMI_H
#define _HMI_H

#include "main.h"

#define  HMI_CMD  11
#define FORMAL_FLOW_ENABLE 0
#define COARSE_TEST_DWELL_MS 1000

enum HmiFlag
{
    None,
    Start,
    Set,
    Abandon,
    Save,
};

typedef enum
{
    StartZoneNone = 0,
    StartZone1,
    StartZone2
} StartZone;

typedef enum
{
    CoarseIdle = 0,
    CoarseStartCorrect,
    CoarseBfsNavigate,
    CoarseStationAlign,
    CoarseStationDwell,
    CoarseFault,
    CoarseFinished,
    CoarseWaitArm
} CoarseStage;

typedef enum
{
    CoarseModeNone = 0,
    CoarseModeTest
} CoarseMode;

enum CoarseTarget
{
    CoarseTargetNone = 0,
    CoarseTargetMaterial = 1,
    CoarseTargetRough = 2,
    CoarseTargetStorage = 3,
    CoarseTargetQr = 4,
    CoarseTargetReturn = 5
};

typedef enum
{
    CoarseFaultNone = 0,
    CoarseFaultImuTimeout,
    CoarseFaultBfsPath,
    CoarseFaultState
} CoarseFaultCode;

typedef struct 
{
    char type;
    uint8_t val;
}vTaskBody;

typedef struct 
{
    uint8_t TaskIndex;
    uint8_t TaskRunIndex;
    uint8_t ItemNum;
    uint8_t PlaceNum;
    uint8_t TaskFlag;
    uint8_t CoarseStage;
    uint8_t CoarseTarget;
    uint8_t CoarseMode;
    int16_t StationYaw;
    uint8_t StartZone;
    uint8_t CoarseTestPassed;
    uint8_t CoarseFaultCode;
    uint8_t StartCorrected;
    vTaskBody TaskBody[10];
}vTask;

enum YoloId
{
    person, bicycle, car, motorcycle, airplane, bus, train, truck, boat, traffic_light, fire_hydrant, stop_sign, parking_meter, bench, bird, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe, backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports_ball, kite, baseball_bat, baseball_glove, skateboard, surfboard, tennis_racket, bottle, wine_glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange, broccoli, carrot, hot_dog, pizza, donut, cake, chair, couch, potted_plant, bed, dining_table, toilet, tv, laptop, mouse, remote, keyboard, cell_phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy_bear, hair_drier, toothbrush
};


extern vTask Task;

extern void Hmi_Init(void);	
extern void Hmi_Idle_Func(void);	
void UpdateShow(uint8_t which,uint8_t num);
void StartMode(uint8_t mode);   // 一键启动模式
void StartCoarseTestMode(void);

#endif
