#include "task.h"

uint8_t task_num = 0;

/*
 * 电机方向单项测试：上电后只执行一次。
 * 每个方向先走5cm，动作之间停500ms，确认方向后再扩大距离。
 */
#if 0
static void Motor_Direction_Test(void)
{
    RunMove(FRONT, 20, 0);
    HAL_Delay(500);

    RunMove(BACK, 20, 0);
    HAL_Delay(500);

    RunMove(LEFT, 20, 0);
    HAL_Delay(500);

    RunMove(RIGHT, 20, 0);
    HAL_Delay(500);

    Stop();
}
#endif

void once_task(void)
{
	/*
	 * === 电机方向测试初始化（已停用）===
	 * Move_Init();
	 * Motor_Direction_Test();
	 */

	/* 恢复正常任务初始化：舵机 -> IMU/PID -> 电机 -> HMI。 */
	Servo_Init();
	HAL_Delay(1000);
	Servo_Watch_To_Move();
	HAL_Delay(2000);
	Servo_Move_To_Plate((uint16_t)JOINT1_PLATE1);
	HAL_Delay(2000);
	/* Camera_Init(); */
	Imu_Init();
	PID_Init();
	Move_Init();
	Hmi_Init();
}

/*
 * === 旧版：while_task 只执行完整任务流程（已停用）===
 * void while_task_OLD(void)
 * {
 *     Procesing();
 * }
 */

/*
 * === 旧版：粗定位和正式任务每轮连续调用（已停用）===
 * 问题：两个状态机共享电机控制接口，粗定位尚未结束时正式任务也可能在同一轮发指令。
 * void while_task(void)
 * {
 *     CoarseProcesing();
 *     Procesing();
 * }
 * === 新版：粗定位未结束时独占主循环，结束后才运行正式任务 ===
 */
void while_task(void)
{
	if (Task.CoarseStage != CoarseIdle)
	{
		CoarseProcesing();
		return;
	}

	/* 粗定位实车验收前，正式搬运状态机保持编译期隔离。 */
#if FORMAL_FLOW_ENABLE
	Procesing();
#endif
}
