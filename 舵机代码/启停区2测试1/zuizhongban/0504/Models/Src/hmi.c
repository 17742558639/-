#include "hmi.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

__IO uint8_t HmiCmd[HMI_CMD] = {0};	
__IO uint8_t HmiCount = 0;	

char set_str[]      = "##set";
char start_str[]    = "##start";
char abandon_str[]  = "##abandon";
char save_str[]     = "##save";
char end_str[]      = "##end";
char exit_str[]     = "##exit";
char tr_str[]       = "##tr";       // transport mode
char is_str[]       = "##is";       // inspection mode
char sp_str[]       = "##sp";       // stop
char auto_str[]     = "##auto";     // 一键启动（兼容HMI旧按钮）
char start_zone1_str[] = "##s1";
char start_zone2_str[] = "##s2";
char test_str[]        = "##test";

vTask Task = {
    .TaskFlag = None,
    .CoarseStage = CoarseIdle,
    .CoarseTarget = CoarseTargetNone,
    .CoarseMode = CoarseModeNone,
    .StartZone = StartZoneNone,
    .CoarseFaultCode = CoarseFaultNone
};

/*
 * === 旧版：一键模式启动任务序列（已停用）===
 * 问题：搬运模式直接展开 A/B/C/D 任务，会在硬件未接入时进入摄像头和机械臂流程。
 * 旧代码保留如下，便于后续恢复完整搬运流程。
 * void StartMode_OLD(uint8_t mode)
 * {
 *     Task.TaskIndex = 0;
 *     Task.TaskRunIndex = 0;
 *     Task.ItemNum = 0;
 *     Task.PlaceNum = 0;
 *
 *     if (mode == 0)
 *     {
 *         Task.TaskBody[0].type = 'w'; Task.TaskBody[0].val = 0;
 *         Task.PlaceNum = 1;
 *         Task.TaskIndex = 1;
 *         Task.TaskRunIndex = 1;
 *         Task.TaskFlag = Start;
 *         return;
 *     }
 *
 *     if (mode == 1)
 *     {
 *         Task.TaskBody[0].type = 'A'; Task.TaskBody[0].val = 1; Task.ItemNum++; Task.TaskIndex++;
 *         Task.TaskBody[1].type = 'w'; Task.TaskBody[1].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[2].type = 'B'; Task.TaskBody[2].val = 1; Task.ItemNum++; Task.TaskIndex++;
 *         Task.TaskBody[3].type = 'x'; Task.TaskBody[3].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[4].type = 'C'; Task.TaskBody[4].val = 1; Task.ItemNum++; Task.TaskIndex++;
 *         Task.TaskBody[5].type = 'y'; Task.TaskBody[5].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[6].type = 'D'; Task.TaskBody[6].val = 1; Task.ItemNum++; Task.TaskIndex++;
 *         Task.TaskBody[7].type = 'z'; Task.TaskBody[7].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *     }
 *     else if (mode == 2)
 *     {
 *         Task.TaskBody[0].type = 'w'; Task.TaskBody[0].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[1].type = 'x'; Task.TaskBody[1].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[2].type = 'y'; Task.TaskBody[2].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[3].type = 'z'; Task.TaskBody[3].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[4].type = 'a'; Task.TaskBody[4].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *         Task.TaskBody[5].type = 'd'; Task.TaskBody[5].val = 0; Task.PlaceNum++; Task.TaskIndex++;
 *     }
 *
 *     Task.TaskFlag = Start;
 * }
 */

/* === 新版：启停区2到原料区的粗定位启动 === */
void StartMode(uint8_t mode)
{
    Task.TaskIndex = 0;
    Task.TaskRunIndex = 0;
    Task.ItemNum = 0;
    Task.PlaceNum = 0;
    Task.CoarseStage = CoarseIdle;
    Task.CoarseTarget = CoarseTargetNone;
    Task.StationYaw = 0;
    Task.StartCorrected = 0;
    memset(Task.TaskBody, 0, sizeof(Task.TaskBody));

    if (mode == 0)
    {
        Task.TaskBody[0].type = 'w';
        Task.PlaceNum = 1;
        Task.TaskIndex = 1;
        Task.TaskRunIndex = 1;
        Task.TaskFlag = Start;
        return;
    }

    if (mode == 1)
    {
        /* Material zone is outside the -Y edge; yaw=0 keeps the arm facing -Y. */
        Task.CoarseTarget = CoarseTargetMaterial;
        Task.StationYaw = 0;
        Task.CoarseStage = CoarseStartCorrect;
        Task.TaskFlag = Start;
        return;
    }

    if (mode == 2)
    {
        Task.TaskBody[0].type = 'w'; Task.PlaceNum++; Task.TaskIndex++;
        Task.TaskBody[1].type = 'x'; Task.PlaceNum++; Task.TaskIndex++;
        Task.TaskBody[2].type = 'y'; Task.PlaceNum++; Task.TaskIndex++;
        Task.TaskBody[3].type = 'z'; Task.PlaceNum++; Task.TaskIndex++;
        Task.TaskBody[4].type = 'a'; Task.PlaceNum++; Task.TaskIndex++;
        Task.TaskBody[5].type = 'd'; Task.PlaceNum++; Task.TaskIndex++;
        Task.TaskFlag = Start;
    }
}

void StartCoarseTestMode(void)
{
    if (Task.StartZone != StartZone1 && Task.StartZone != StartZone2)
        return;

    Task.TaskIndex = 0;
    Task.TaskRunIndex = 0;
    Task.ItemNum = 0;
    Task.PlaceNum = 0;
    Task.CoarseMode = CoarseModeTest;
    Task.CoarseTarget = CoarseTargetQr;
    Task.CoarseStage = CoarseStartCorrect;
    Task.StationYaw = 0;
    Task.CoarseTestPassed = 0;
    Task.CoarseFaultCode = CoarseFaultNone;
    Task.StartCorrected = 0;
    memset(Task.TaskBody, 0, sizeof(Task.TaskBody));
    Task.TaskFlag = Start;
}

static void HmiSelectStartZone(uint8_t start_zone)
{
    /* 运行中不允许HMI改变起点，防止BFS当前位置与物理位置脱节。 */
    if (Task.TaskFlag == Start ||
        (Task.CoarseStage != CoarseIdle &&
         Task.CoarseStage != CoarseFault &&
         Task.CoarseStage != CoarseFinished))
        return;

    Task.StartZone = start_zone;
    Task.TaskFlag = None;
    Task.CoarseStage = CoarseIdle;
    Task.CoarseTarget = CoarseTargetNone;
    Task.CoarseMode = CoarseModeNone;
    Task.CoarseFaultCode = CoarseFaultNone;
}

static uint8_t HmiCommandEquals(const char *command, uint8_t length)
{
    if (HmiCount != length)  //防止字节越界比较
        return 0;
    return memcmp((const void *)HmiCmd, command, length) == 0;  //用于逐字节比较两块内存区域的内容。
}

/*
 * === 旧版：HMI可直接启动正式任务及兼容模式（已停用）===
 * 问题：当前阶段只允许双启停区粗定位，旧指令可能绕过##test互锁并驱动车辆。
 * === 新版：旧实现完整保留在条件编译块中 ===
 */
#if 0
void Hmi_Func(void)
{
    switch(Task.TaskFlag)
    {
        case None:
            /* === 新模式指令（4字节）=== */
            if(HmiCount == 4)
            {
                if(HmiCmd[0] == '#' && HmiCmd[1] == '#')
                {
                    if(HmiCmd[2] == 't' && HmiCmd[3] == 'r')      StartMode(1);
                    else if(HmiCmd[2] == 'i' && HmiCmd[3] == 's') StartMode(2);
                    else if(HmiCmd[2] == 's' && HmiCmd[3] == 'p') StartMode(0);
                    else memset((void*)HmiCmd,0,HMI_CMD);
                }
            }
            /* === ##auto（6字节，兼容HMI已有按钮）=== */
            else if(HmiCount == 6)
            {
                for(uint8_t i = 0;i < HmiCount ;i++)
                {
                    if(HmiCmd[i] != auto_str[i])
                    {
                        memset((void*)HmiCmd,0,HMI_CMD);
                        return;
                    }
                }
                StartMode(1);
            }
            /* === 旧指令（兼容HMI现有按钮）=== */
            else if(HmiCount == 5)
            {
                for(uint8_t i = 0;i < HmiCount ;i++)
                {
                    if(HmiCmd[i] != set_str[i])
                    {
                        memset((void*)HmiCmd,0,HMI_CMD);
                        return;
                    }
                }
                Task.TaskFlag = Set;
                Task.StartCorrected = 0;
            }
            else if(HmiCount == 7)
            {
                for(uint8_t i = 0;i < HmiCount ;i++)
                {
                    if(HmiCmd[i] != start_str[i])
                    {
                        memset((void*)HmiCmd,0,HMI_CMD);
                        return;
                    }
                }
                Task.TaskFlag = Start;
            }
            break;

        case Set:
            if(HmiCount == 3) // 地点标识 ##x
            {
                if(HmiCmd[0] == '#' && HmiCmd[1] == '#')
                {
                    if(Task.TaskIndex >= 10) { memset((void*)HmiCmd,0,HMI_CMD); return; }
                    Task.TaskBody[Task.TaskIndex].type = HmiCmd[HmiCount-1];
                    Task.TaskBody[Task.TaskIndex].val = 0;
                    Task.TaskIndex++;
                    Task.PlaceNum++;
                }
            }
            else if(HmiCount == 4)  //物品标识 ##X0
            {
                if(HmiCmd[0] == '#' && HmiCmd[1] == '#')
                {
                    if(Task.TaskIndex >= 10) { memset((void*)HmiCmd,0,HMI_CMD); return; }
                    Task.TaskBody[Task.TaskIndex].type = HmiCmd[2];
                    Task.TaskBody[Task.TaskIndex].val = HmiCmd[3] - '0';
                    Task.TaskIndex++;
                    Task.ItemNum++;
                }
            }
            else if(HmiCount == 6)  // ##exit
            {
                for(uint8_t i = 0;i < HmiCount ;i++)
                {
                    if(HmiCmd[i] != exit_str[i])
                    {
                        memset((void*)HmiCmd,0,HMI_CMD);
                        return;
                    }
                }
                Task.TaskFlag = None;
            }
            else if(HmiCount == 7)
            {
                for(uint8_t i = 0;i < HmiCount ;i++)
                {
                    if(HmiCmd[i] != start_str[i])
                    {
                        memset((void*)HmiCmd,0,HMI_CMD);
                        return;
                    }
                }
                Task.TaskFlag = Start;
            }
            break;
        default:
            break;
    }
}
#endif

/*
 * === 旧版：粗定位只接受##test启动（已停用）===
 * 问题：HMI实际“开始”按钮发送##start，活动函数忽略该指令，
 *       造成选择S1/S2后点击开始仍停留在CoarseIdle，车辆不动。
 * void Hmi_Func(void)
 * {
 *     if (HmiCommandEquals(start_zone1_str, 4))
 *         HmiSelectStartZone(StartZone1);
 *     else if (HmiCommandEquals(start_zone2_str, 4))
 *         HmiSelectStartZone(StartZone2);
 *     else if (HmiCommandEquals(test_str, 6))
 *     {
 *         if (Task.StartZone != StartZone1 && Task.StartZone != StartZone2)
 *             return;
 *         if (Task.TaskFlag == None && Task.CoarseStage == CoarseIdle)
 *             StartCoarseTestMode();
 *     }
 *     memset((void *)HmiCmd, 0, HMI_CMD);
 * }
 */

/*
 * === 上一版：S1/S2选择后##start和##test都启动（已停用）===
 * 问题：当前工程是粗定位测试工程，普通##start不应绕过测试入口。
 */
#if 0
void Hmi_Func(void)
{
    uint8_t start_command;

    if (HmiCommandEquals(start_zone1_str, 4))
    {
        HmiSelectStartZone(StartZone1);
    }
    else if (HmiCommandEquals(start_zone2_str, 4))
    {
        HmiSelectStartZone(StartZone2);
    }

    start_command = (uint8_t)(HmiCommandEquals(start_str, 7) ||
                               HmiCommandEquals(test_str, 6));
    if (start_command)
    {
        /* 未选择S1/S2时拒绝启动，避免没有合法BFS起点就驱动车辆。 */
        if ((Task.StartZone == StartZone1 || Task.StartZone == StartZone2) &&
            Task.TaskFlag == None && Task.CoarseStage == CoarseIdle)
        {
            StartCoarseTestMode();
        }
    }

    memset((void *)HmiCmd, 0, HMI_CMD);
}
#endif

/* 当前测试版：S1/S2只选择起点，必须发送##test才启动粗定位。 */
void Hmi_Func(void)
{
    if (HmiCommandEquals(start_zone1_str, 4))
    {
        HmiSelectStartZone(StartZone1);
    }
    else if (HmiCommandEquals(start_zone2_str, 4))
    {
        HmiSelectStartZone(StartZone2);
    }
    else if (HmiCommandEquals(test_str, 6))
    {
        if (Task.StartZone != StartZone1 && Task.StartZone != StartZone2)
        {
            memset((void *)HmiCmd, 0, HMI_CMD);
            return;
        }

        if (Task.TaskFlag == None && Task.CoarseStage == CoarseIdle)
        {
            StartCoarseTestMode();
        }
    }
    else
    {
        /* 测试粗定位阶段忽略##start，避免误启动正式流程。 */
    }

    memset((void *)HmiCmd, 0, HMI_CMD);
}

void Hmi_Init(void)
{

	__HAL_UART_CLEAR_IDLEFLAG(&huart6); 						
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); 				
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)HmiCmd, HMI_CMD); 
}

char a[30];
void UpdateShow(uint8_t which,uint8_t num)
{
    switch (which)
    {
        case scissors:
            sprintf(a,"t0.txt=\"scissors\"\xff\xff\xff");
            HAL_UART_Transmit_DMA(&huart6,(uint8_t*)a,20);
            while (!__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC)); 
            break;
    }
    memset((uint8_t*)a,0,30);
    HAL_Delay(100);
    sprintf(a,"t1.txt=\"%d\"\xff\xff\xff",num);
    HAL_UART_Transmit_DMA(&huart6,(uint8_t*)a,13);
    while (!__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC)); 
}

void Hmi_Idle_Func(void)
{
    if(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6); 		

		HAL_UART_DMAStop(&huart6); 								
		 
		HmiCount = HMI_CMD - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);	
        
        Hmi_Func();
		
		HAL_UART_Receive_DMA(&huart6, (uint8_t *)HmiCmd, HMI_CMD);
	}
}
