#include "move.h"
#include <math.h>

/*
电机ID：

	^y
	|
	|	3   前   1
	|
	|
	|	2        4
	----------------->x
	顺时针 +

	x: 146cm   5.9s  60=24.7cm/s
	y: 17cm	   0.7s	 60=24.3cm/s

*/

__IO uint8_t MoveCmd[Move_CMD] = {0};
__IO uint8_t MoveCount = 0;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

xMOVE pxMove;
xMOVETAR pxMoveTar;

uint32_t my_abs(int data)
{
	if(data < 0) return -data;
	else return data;
}

/* 自转函数声明 */
void Rotate(uint16_t angle_deg, uint8_t dir);
void Move_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)MoveCmd, Move_CMD);
	HAL_Delay(2000);

	/* --- 清TC + 等电机准备好 --- */
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
	HAL_Delay(100);

	/* --- 强制切4个电机到串口模式 + 保存Flash --- */
	Emm_V5_Modify_Ctrl_Mode(1, true, 0);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
	Emm_V5_Modify_Ctrl_Mode(2, true, 0);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
	Emm_V5_Modify_Ctrl_Mode(3, true, 0);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
	Emm_V5_Modify_Ctrl_Mode(4, true, 0);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));

	/* --- 使能4个电机 --- */
	Emm_V5_En_Control(1, true, false);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
	Emm_V5_En_Control(2, true, false);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
	Emm_V5_En_Control(3, true, false);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));
	Emm_V5_En_Control(4, true, false);
	while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC));


		/* 测试路径已禁用 */
}

/* ============================================================
   自转函数 —— 用 Pos_Control 原地旋转
   实测：2000脉冲=45° → 1°≈44.4脉冲 → 180°≈8000脉冲
   dir: 0=顺时针(CW), 1=逆时针(CCW)
   ============================================================ */
#define ROTATE_VEL 45

void Rotate(uint16_t angle_deg, uint8_t dir)
{
	uint32_t pulses = (uint32_t)((float)angle_deg * 53.33f);

	HAL_Delay(50);
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);

	/*
	 * === 旧版：四个电机收到指令后依次立即启动（已弃用）===
	 * 问题：地址1、2先启动，进入旋转时会出现短暂不同步。
	 * for (uint8_t i = 0; i < 4; i++)
	 * {
	 *     Emm_V5_Pos_Control(i + 1, dir, 78, 5, pulses, 0, 0);
	 *     HAL_Delay(10);
	 * }
	 * === 新版：先缓存四帧，再广播同步启动 ===
	 */
	for (uint8_t i = 0; i < 4; i++)
	{
		/* 旋转降速，减小麦轮打滑和四轮响应差造成的转动中心偏移。 */
		Emm_V5_Pos_Control(i + 1, dir, ROTATE_VEL, 5, pulses, 0, 1);
		HAL_Delay(10);
	}
	Emm_V5_Synchronous_motion(0);

	/* 按当前旋转速度估算执行时间，并额外留出机械停稳时间。 */
	uint32_t wait_ms = (uint32_t)((float)pulses /
	                    ((float)PULSES_PER_REV * (float)ROTATE_VEL / 60.0f) * 1000.0f) + 400;
	HAL_Delay(wait_ms);
}

void Move_Idle_Func(void)
{
	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		MoveCount = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		HAL_UART_Receive_DMA(&huart1, (uint8_t *)MoveCmd, Move_CMD);
	}
}


static void RotateMoveTarSet(int omega)
{
	pxMoveTar.acc = 0;
	pxMoveTar.x = 0;
	pxMoveTar.y = 0;
	pxMoveTar.omega = omega;
}

static void RunMoveTarSet(int x,int y)
{
	pxMoveTar.acc = 0;
	pxMoveTar.x = x;
	pxMoveTar.y = y;
	pxMoveTar.omega = pidout_i(&imu_pid,yaw);
}

static void MoveTarClear(void)
{
	pxMoveTar.x = 0;
	pxMoveTar.y = 0;
	pxMoveTar.omega = 0;
}

static void MoveSolve(void)
{
	int w[4];

	w[0] =  pxMoveTar.y - pxMoveTar.x - pxMoveTar.omega;
	w[1] =  -pxMoveTar.y + pxMoveTar.x - pxMoveTar.omega;
	w[2] = (-1) * (pxMoveTar.y + pxMoveTar.x + pxMoveTar.omega);
	w[3] = (-1) * (-pxMoveTar.y - pxMoveTar.x + pxMoveTar.omega);

	for(uint8_t i = 0;i < 4;i++)
	{
		if(w[i]<0)
		{
			pxMove.dir[i] = 1;    /* w<0→CCW，跟RunMove统一 */
			w[i] *= -1;
		}else pxMove.dir[i] = 0;  /* w>=0→CW */
		pxMove.vel[i] = w[i];
	}
	pxMove.acc = pxMoveTar.acc;

	for(uint8_t i = 0;i < 4;i++)
	{
		Emm_V5_Vel_Control(i+1, pxMove.dir[i], pxMove.vel[i], pxMove.acc, 0);  // snF=0 立即执行
		HAL_Delay(5);
	}
}

void Stop(void)
{
	MoveTarClear();
	MoveSolve();
}

void RotateMove(int omega)
{
	RotateMoveTarSet(omega);
	MoveSolve();
}
/*
 * ============================================================
 *  【新版】位置模式闭环走 —— 告诉电机"转多少脉冲"，电机自己数
 *   原理：不再发速度指令+掐表，而是直接发位置指令
 *   电机内部有编码器，自己知道转了多少，转到目标自动停
 *
 *   换算公式：
 *     目标脉冲 = 距离(cm) ÷ (π × 轮子直径) × 每圈脉冲数
 *
 *   举例：向前走 50cm
 *     轮子周长 = 3.14159 × 8 = 25.13cm
 *     需要转 50 ÷ 25.13 ≈ 2 圈
 *     每圈 1600 脉冲(200整步×8细分) → 2 × 1600 = 3200 脉冲
 *     告诉电机"你转 3200 个脉冲"，电机内部闭环执行
 * ============================================================
 */
void RunMove(uint8_t dir, uint16_t distance, uint8_t isrunmap)
{
	/* --- 清TC标志，防上次残留导致指令碰撞 --- */
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);

	/* --- 第1步：把厘米换算成脉冲数 --- */
	uint32_t pulses = CM_TO_PULSES(distance);

	/* --- 第2步：判断方向（和旧版逻辑一致）--- */
	int x_sign = 0;   // x方向符号：+1=右, -1=左, 0=不沿x走
	int y_sign = 0;   // y方向符号：+1=前, -1=后, 0=不沿y走

	if(isrunmap)
	{
		/*
		 * === 旧版：地图方向直接使用旧横移符号（已停用）===
		 * 问题：实测S2从(0,4)前往(0,2)时，车辆已正确顺时针转到yaw=-90，
		 *       但BACK使用x_sign=+1，最终沿地图+Y开出场地；同时yaw=0时
		 *       RIGHT实测对应地图-X，与代码坐标x+相反。
		 * switch (dir)
		 * {
		 *     case RIGHT: x_sign =  1; y_sign = 0; break;
		 *     case LEFT:  x_sign = -1; y_sign = 0; break;
		 *     case FRONT: x_sign = -1; y_sign = 0; break;
		 *     case BACK:  x_sign =  1; y_sign = 0; break;
		 * }
		 * === 新版：依据实际地图(+X向左、+Y向下)和实车方向反转横移符号 ===
		 */
		switch (dir)
		{
			case RIGHT: x_sign = -1; y_sign = 0; break; /* 地图x+：yaw=0时使用实车LEFT */
			case LEFT:  x_sign =  1; y_sign = 0; break; /* 地图x-：yaw=0时使用实车RIGHT */
			case FRONT: x_sign =  1; y_sign = 0; break; /* 地图y+：yaw=-90后的相反横移 */
			case BACK:  x_sign = -1; y_sign = 0; break; /* 地图y-：S2到二维码，修正+Y反向 */
		}
	}
	else
	{
		/* 普通模式：直接在车体坐标系下走 */
		switch (dir)
		{
			case RIGHT:
				x_sign =  1;    // 车体坐标向右
				y_sign =  0;
				break;

			case LEFT:
				x_sign = -1;    // 车体坐标向左
				y_sign =  0;
				break;

			case FRONT:
				x_sign =  0;
				y_sign =  1;    // 车体坐标向前
				break;

			case BACK:
				x_sign =  0;
				y_sign = -1;    // 车体坐标向后
				break;
		}
	}

	/* --- 第3步：麦克纳姆轮分解 ---
	   把"车子往哪个方向走"翻译成"4个轮子各自往哪转"

	   轮子编号（俯视图）：
	         ↑ y(前)
	         |
	    ③   |   ①
	         |
	   ------+------→ x(右)
	         |
	    ②   |   ④
	         |

	   公式：
	     w1 =  y - x      1号轮：前+左 为正转
	     w2 = -y + x      2号轮：后+右 为正转
	     w3 = -(y + x)    3号轮：前+右 为反转（取负）
	     w4 = -(-y - x)   4号轮：后+左 为反转（取负）

	   举例：纯前进（y_sign=1, x_sign=0）
	     w1 =  1  →  正转
	     w2 = -1  →  反转
	     w3 = -1  →  反转
	     w4 =  1  →  正转
	     四个轮同速，1和4正转、2和3反转 → 车直直往前走

	   举例：纯右移（y_sign=0, x_sign=1）
	     w1 = -1  →  反转
	     w2 =  1  →  正转
	     w3 = -1  →  反转
	     w4 =  1  →  正转
	     四个轮同速，2和4正转、1和3反转 → 车横着右移
	   -------------------------------------------------------- */
	int w[4];   // 四个轮子的转动方向和幅度（+正转，-反转）
	/*
	 * === 旧版：横向分量被错误合并（已停用）===
	 * 问题：w[1]~w[3]都只使用(y_sign-x_sign)，导致RIGHT与BACK的
	 * 四个地址方向完全相同，无法独立横移。
	 * w[0] = y_sign - x_sign;
	 * w[1] = -(y_sign - x_sign);
	 * w[2] = -(y_sign - x_sign);
	 * w[3] = y_sign - x_sign;
	 */

	/* 新版：按麦克纳姆轮完整分解，地址顺序保持1、2、3、4。 */
	w[0] =  y_sign - x_sign;        // 地址1：前右轮
	w[1] = -y_sign + x_sign;        // 地址2：后左轮
	w[2] = -(y_sign + x_sign);      // 地址3：前左轮
	w[3] = -(-y_sign - x_sign);     // 地址4：后右轮

	/*
	 * === 右移方向修正（已回退）===
	 * 按要求恢复原始电机映射，暂不对RIGHT分支单独反转3、4号轮。
	 * if (dir == RIGHT)
	 * {
	 *     w[2] = -w[2];
	 *     w[3] = -w[3];
	 * }
	 */

	/* --- 第4步：记下出发朝向，走完后校准 --- */
	short ref_yaw = yaw;

	/* --- 第5步：给4个电机发位置指令 --- */
	HAL_Delay(50);

	/*
	 * === 旧版：四个电机收到指令后依次立即启动（已弃用）===
	 * 问题：地址1、2比地址3、4先启动约20~30ms，起步瞬间不同步。
	 * for (uint8_t i = 0; i < 4; i++)
	 * {
	 *     uint8_t dir_cmd = (w[i] >= 0) ? 0 : 1;
	 *     Emm_V5_Pos_Control(i + 1, dir_cmd, RUNMOVE_VEL, 5,
	 *                        pulses, 0, 0);
	 *     HAL_Delay(10);
	 * }
	 * === 新版：四个位置指令缓存后，由地址0广播同步触发 ===
	 */
	for (uint8_t i = 0; i < 4; i++)
	{
		uint8_t dir_cmd = (w[i] >= 0) ? 0 : 1;
		Emm_V5_Pos_Control(i + 1,
		                   dir_cmd,
		                   RUNMOVE_VEL,
		                   5,
		                   pulses,
		                   0,
		                   1);
		HAL_Delay(10);
	}
	Emm_V5_Synchronous_motion(0);

	/* --- 第6步：等待走完 --- */
	uint32_t wait_ms = (uint32_t)((float)pulses / (float)PULSES_PER_REV * 1000.0f) + 200;
	HAL_Delay(wait_ms);

	/* --- 第7步：陀螺仪校准 —— 走完后方向偏了就修正 --- */
	HAL_Delay(80);
	if (my_abs(ref_yaw - yaw) > 2)
	{
		Rotation(ref_yaw);  // IMU闭环修正回出发时的朝向
	}
}

void PidMove(int x,int y)
{
	RunMoveTarSet(x,y);
	MoveSolve();
}

/*
 * ============================================================
 *  带IMU实时修正的直线移动（仿冠军 Chassis_MoveOnce）
 *  跟 RunMove 的区别：
 *    RunMove：发位置指令 → 掐表等 → 走完检查IMU → 歪了再修
 *    本函数：发速度指令 → 每30ms读IMU → 歪了立刻修 → 全程闭环
 * ============================================================
 */
void RunMoveWithIMU(uint8_t dir, uint16_t distance)
{
	int x_sign = 0, y_sign = 0;

	/* --- 第1步：方向分解 --- */
	switch (dir)
	{
		case FRONT: x_sign =  0; y_sign =  1; break;
		case BACK:  x_sign =  0; y_sign = -1; break;
		case LEFT:  x_sign = -1; y_sign =  0; break;
		case RIGHT: x_sign =  1; y_sign =  0; break;
	}

	/* --- 第2步：估算走多久 ---
	   40 RPM = 0.67圈/秒，轮子周长 25.13cm，车速 ≈ 16.7cm/s
	   时间(ms) = 距离(cm) ÷ 16.7 × 1000
	   -------------------------------------------------------- */
	int max_speed = 40;  /* 40 RPM，比位置模式慢一点，平稳 */
	float speed_cm_per_s = WHEEL_CIRCUMFERENCE * (float)max_speed / 60.0f;
	uint32_t duration_ms = (uint32_t)((float)distance / speed_cm_per_s * 1000.0f);
	uint32_t accel_ms   = 200;   /* 加速段时间 200ms */
	uint32_t decel_ms   = 200;   /* 减速段时间 200ms */
	uint32_t const_ms;

	if (duration_ms > accel_ms + decel_ms)
		const_ms = duration_ms - accel_ms - decel_ms;
	else
	{
		/* 距离太短，加减速各占一半，没有匀速段 */
		accel_ms = duration_ms / 2;
		decel_ms = duration_ms - accel_ms;
		const_ms = 0;
	}

	/* --- 第3步：记下出发朝向，设IMU目标 --- */
	short ref_yaw = yaw;
	imu_pid.desired = ref_yaw;
	IMUPidClear();

	uint32_t dt = 30;  /* 每30ms修正一次 */

	/* --- 第4步：加速段（正弦曲线，平滑起步）--- */
	uint32_t accel_steps = accel_ms / dt;
	if (accel_steps > 0)
	{
		for (uint32_t i = 0; i < accel_steps; i++)
		{
			float ratio = 0.5f - 0.5f * cosf(3.14159f * (float)i / (float)accel_steps);
			int vx = (int)((float)(x_sign * max_speed) * ratio);
			int vy = (int)((float)(y_sign * max_speed) * ratio);
			PidMove(vx, vy);   /* IMU PID 在 PidMove 内部自动生效 */
			HAL_Delay(dt);
		}
	}

	/* --- 第5步：匀速段（IMU全程盯着）--- */
	uint32_t const_steps = const_ms / dt;
	for (uint32_t i = 0; i < const_steps; i++)
	{
		PidMove(x_sign * max_speed, y_sign * max_speed);
		HAL_Delay(dt);
	}

	/* --- 第6步：减速段（正弦曲线，平滑停车）--- */
	uint32_t decel_steps = decel_ms / dt;
	if (decel_steps > 0)
	{
		for (uint32_t i = 0; i < decel_steps; i++)
		{
			float ratio = 0.5f + 0.5f * cosf(3.14159f * (float)i / (float)decel_steps);
			int vx = (int)((float)(x_sign * max_speed) * ratio);
			int vy = (int)((float)(y_sign * max_speed) * ratio);
			PidMove(vx, vy);
			HAL_Delay(dt);
		}
	}

	/* --- 第7步：停车 --- */
	Stop();

	/* --- 第8步：最后校准 --- */
	HAL_Delay(80);
	if (my_abs(ref_yaw - yaw) > 2)
	{
		Rotation(ref_yaw);
	}
}

/*
 * === 旧版：速度模式+PID自转（已弃用）===
 * 问题：依赖 Vel_Control，1号电机不响应速度指令，自转失败
void Rotation_OLD(short angle_desired)
{
	int omega = 0;
	float pidout_imu = 0;
	uint8_t wait_index = 0;

	yaw_desired = angle_desired;
	imu_pid.desired = yaw_desired;

	while(wait_index < 10)
	{
		if(my_abs(yaw - yaw_desired) > 1)
		{
			wait_index = 0;
			pidout_imu = pidout_i(&imu_pid,yaw);
			omega = (int)pidout_imu;
			RotateMove(omega);
			HAL_Delay(10);
		}else
		{
			wait_index++;
			HAL_Delay(20);
		}
	}
	Stop();
}
 * === 新版：陀螺仪闭环自转 ===
 * 读IMU角度 → 算误差 → 用Rotate(Pos_Control)逐步修正 → 到位停
 * 优点：不打滑、不受电压影响、自动补偿
 */

static short DirToYaw(uint8_t dir)
{
	if (dir == RIGHT || dir == LEFT)  return 0;
	return -90;
}

/*
 * === 后续版：保持工位朝向并在BFS拐角提前对准（已回退）===
 * 回退目标：恢复到“每走82cm由陀螺仪检查并修正姿态”的版本。
 * static uint8_t CoarseTravelYawValid = 0;
 * static short CoarseTravelYaw = 0;
 * static uint8_t CoarseDestinationYawReady = 0;
 */

static int32_t NormalizeAngleError(int32_t error);

/*
 * === 旧版：TurnIMU最多补偿两次且直接读取旧yaw（已停用）===
 * 问题：Rotate返回后没有等待新IMU帧，BFS可能误判已对准并立即开始平移。
 */
#if 0
void TurnIMU(short delta_deg)
{
	/*
	 * === 旧版：用short保存相对目标和误差（已停用）===
	 * 问题：累计yaw接近short边界时，yaw + delta_deg会溢出，
	 * 归一化前已经丢失真实目标角度。
	 * short target = yaw + delta_deg;
	 * short error = target - yaw;
	 * === 新版：先用int32_t计算，再归一化最短误差 ===
	 */
	int32_t target = (int32_t)yaw + (int32_t)delta_deg;
	int32_t error = NormalizeAngleError(target - (int32_t)yaw);
	uint8_t d;
	uint16_t step;

	if (my_abs(error) <= 2) return;

	d = (error > 0) ? 1 : 0;
	step = (uint16_t)((float)my_abs(error) * 0.9f);
	if (step < 2) step = 2;
	Rotate(step, d);
	HAL_Delay(80);

	error = NormalizeAngleError(target - (int32_t)yaw);

	if (my_abs(error) >= 1)
	{
		d = (error > 0) ? 1 : 0;
		step = (uint16_t)(my_abs(error) + 1);
		if (step < 1) step = 1;
		Rotate(step, d);
		HAL_Delay(80);
	}
}
#endif

/*
 * === 旧版：Rotation直接使用累计yaw计算误差（已弃用）===
 * 问题：yaw跨圈累计后，目标0度与当前720度会得到-720度误差，
 *       车辆会反转多余的整圈；IMU异常回退也没有根据当前朝向选最短方向。
void Rotation(short angle_desired)
{
	short error;
	short yaw_start, yaw_after;
	uint8_t stable_cnt = 0;
	uint16_t step;
	uint8_t dir;
	static uint8_t imu_ok = 0;        // static记忆,第一次检测后保存结果

	yaw_desired = angle_desired;
	HAL_Delay(50);

	// --- 首次调用时检测IMU是否在工作 ---
	if (imu_ok == 0)
	{
		yaw_start = yaw;
		Rotate(5, 0);                 // 转5°试试，确保IMU能检测到变化
		yaw_after = yaw;

		if (my_abs(yaw_after - yaw_start) > 2)
				imu_ok = 1;               // IMU正常
		else
			imu_ok = 2;               // IMU无效,走计时模式
	}

	if (imu_ok == 1)
	{
		// === IMU闭环模式 ===
		short last_error = 0;

		while (stable_cnt < 2)
		{
			error = yaw_desired - yaw;

			if (my_abs(error) > 3)       // 容差±3°，转不到位不舒服
			{
				stable_cnt = 0;

				// yaw=CCW+，车=CW+，方向取反：error>0→需CCW→dir=1(CCW)
				dir = (error > 0) ? 1 : 0;

				// 自适应步长：大步追、小步调
				if (last_error != 0 && ((error > 0) != (last_error > 0)))
					step = (uint16_t)((float)my_abs(error) * 0.4f);  // 冲过头了，30%慢慢回
				else
					step = (uint16_t)((float)my_abs(error) * 0.8f);  // 正常追，60%修正

				if (step < 1)  step = 1;    // 最小1°，精细修正
				if (step > 90) step = 90;   // 最多90°，大步走不卡顿

				last_error = error;
				Rotate(step, dir);
			}
			else
			{
				stable_cnt++;
				HAL_Delay(20);
			}
		}

		// --- 最终微调：误差>1°就再补一刀 ---
		error = yaw_desired - yaw;
		if (my_abs(error) >= 1)
		{
			dir = (error > 0) ? 1 : 0;
			step = (uint16_t)(my_abs(error));
			if (step < 1) step = 1;
			Rotate(step, dir);
			HAL_Delay(100);
		}
	}
	else
	{
		// === IMU无效，走计时模式 ===
		short ad = angle_desired;
		if (ad < 0) ad = -ad;
		Rotate((uint16_t)ad, 0);
	}
}
 * === 新版：Rotation最短角闭环 + 防失控保护 ===
 * 改了什么：每次计算都把误差归一化到[-180,180]，使用int32_t避免short减法溢出；
 *           限制单次调用的修正次数和累计指令角，IMU异常时也按最短方向回退。
 */
#define ROTATION_MAX_CORRECTIONS    12
#define ROTATION_MAX_TOTAL_DEGREES  270
#define ROTATION_MAX_STEP_DEGREES   30
#define ROTATION_IMU_TIMEOUT_MS     500

static int32_t NormalizeAngleError(int32_t error)
{
	while (error > 180)  error -= 360;
	while (error < -180) error += 360;
	return error;
}

static void RotationStopAll(void)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		Emm_V5_Stop_Now(i + 1, false);
		HAL_Delay(10);
	}
}

/*
 * === 上一版：缓存IMU健康状态并允许开环回退（已停用）===
 * 问题：首次5°检测若未及时收到新帧，imu_ok会永久变成2；此后所有转向
 *       都不再检查陀螺仪到位，只按脉冲开环旋转。
 */
#if 0
void Rotation(short angle_desired)
{
	int32_t error;
	int32_t last_error = 0;
	/* 旧版：short yaw_start/yaw_after（已停用，跨圈比较可能溢出） */
	int32_t yaw_start, yaw_after;
	uint8_t stable_cnt = 0;
	uint8_t correction_cnt = 0;
	uint16_t step;
	uint16_t total_degrees = 0;
	uint8_t dir;
	static uint8_t imu_ok = 0;

	yaw_desired = angle_desired;
	HAL_Delay(50);

	/*
	 * 目标已经在容差内时不做首次5°健康检测，避免Rotation(0)产生额外转动。
	 * 真正需要转向时仍保留一次IMU健康检测和无IMU回退路径。
	 */
	error = NormalizeAngleError((int32_t)angle_desired - (int32_t)yaw);
	if (my_abs(error) <= 2)
		return;

	if (imu_ok == 0)
	{
		yaw_start = yaw;
		Rotate(5, 0);
		yaw_after = yaw;

		if (my_abs(yaw_after - yaw_start) > 2)
			imu_ok = 1;
		else
			imu_ok = 2;
	}

	if (imu_ok == 1)
	{
		while (stable_cnt < 2 && correction_cnt < ROTATION_MAX_CORRECTIONS)
		{
			error = NormalizeAngleError((int32_t)yaw_desired - (int32_t)yaw);

			if (my_abs(error) > 3)
			{
				stable_cnt = 0;
				dir = (error > 0) ? 1 : 0;

				if (last_error != 0 && ((error > 0) != (last_error > 0)))
					step = (uint16_t)((float)my_abs(error) * 0.4f);
				else
					step = (uint16_t)((float)my_abs(error) * 0.8f);

				if (step < 1)  step = 1;
				if (step > 90) step = 90;

				if ((uint32_t)total_degrees + step > ROTATION_MAX_TOTAL_DEGREES)
				{
					RotationStopAll();
					return;
				}

				last_error = error;
				total_degrees += step;
				correction_cnt++;
				Rotate(step, dir);
			}
			else
			{
				stable_cnt++;
				HAL_Delay(20);
			}
		}

		if (stable_cnt < 2)
		{
			RotationStopAll();
			return;
		}

		error = NormalizeAngleError((int32_t)yaw_desired - (int32_t)yaw);
		if (my_abs(error) >= 1)
		{
			dir = (error > 0) ? 1 : 0;
			step = (uint16_t)my_abs(error);
			if (step < 1) step = 1;

			if ((uint32_t)total_degrees + step > ROTATION_MAX_TOTAL_DEGREES)
			{
				RotationStopAll();
				return;
			}

			Rotate(step, dir);
			HAL_Delay(100);
		}
	}
	else
	{
		error = NormalizeAngleError((int32_t)angle_desired - (int32_t)yaw);
		if (my_abs(error) <= 2) return;

		dir = (error > 0) ? 1 : 0;
		step = (uint16_t)my_abs(error);
		Rotate(step, dir);
	}
}
#endif

/*
 * === 新版：每一步都使用新鲜IMU帧的闭环转向 ===
 * 改了什么：取消永久缓存和开环回退；转前、转后均等待连续有效新帧；
 *           单次最多修正30°，IMU超时或连续两次无角度响应立即停车报失败。
 */
uint8_t Rotation(short angle_desired)
{
	int32_t error;
	int32_t last_error = 0;
	int32_t yaw_before;
	int32_t yaw_after;
	uint8_t stable_cnt = 0;
	uint8_t correction_cnt = 0;
	uint8_t no_response_cnt = 0;
	uint16_t step;
	uint16_t total_degrees = 0;
	uint8_t dir;

	yaw_desired = angle_desired;

	/* 不接受旧角度：转向决策前必须重新收到连续有效帧。 */
	if (!Imu_Wait_Ready(ROTATION_IMU_TIMEOUT_MS))
	{
		RotationStopAll();
		return 0;
	}

	while (stable_cnt < 2 && correction_cnt < ROTATION_MAX_CORRECTIONS)
	{
		error = NormalizeAngleError((int32_t)yaw_desired - (int32_t)yaw);

		if (my_abs(error) <= 2)
		{
			stable_cnt++;
			if (stable_cnt >= 2)
				break;

			/* 第二次到位判断必须来自另一组新帧。 */
			if (!Imu_Wait_Ready(ROTATION_IMU_TIMEOUT_MS))
			{
				RotationStopAll();
				return 0;
			}
			continue;
		}

		stable_cnt = 0;
		dir = (error > 0) ? 1 : 0;

		if (last_error != 0 && ((error > 0) != (last_error > 0)))
			step = (uint16_t)((float)my_abs(error) * 0.4f);
		else
			step = (uint16_t)((float)my_abs(error) * 0.7f);

		if (step < 2) step = 2;
		if (step > ROTATION_MAX_STEP_DEGREES) step = ROTATION_MAX_STEP_DEGREES;

		if ((uint32_t)total_degrees + step > ROTATION_MAX_TOTAL_DEGREES)
		{
			RotationStopAll();
			return 0;
		}

		yaw_before = yaw;
		last_error = error;
		total_degrees += step;
		correction_cnt++;
		Rotate(step, dir);

		/* 电机动作结束后必须等到新的IMU数据，禁止用旧yaw继续判断。 */
		if (!Imu_Wait_Ready(ROTATION_IMU_TIMEOUT_MS))
		{
			RotationStopAll();
			return 0;
		}

		yaw_after = yaw;
		if (step >= 5 && my_abs(NormalizeAngleError(yaw_after - yaw_before)) < 1)
			no_response_cnt++;
		else
			no_response_cnt = 0;

		if (no_response_cnt >= 2)
		{
			RotationStopAll();
			return 0;
		}
	}

	if (stable_cnt < 2)
	{
		RotationStopAll();
		return 0;
	}

	RotationStopAll();
	return 1;
}

uint8_t TurnIMU(short delta_deg)
{
	int32_t target = (int32_t)yaw + (int32_t)delta_deg;
	return Rotation((short)target);
}

static void PosTarSetVal(int x,int y)
{
	Path.pos_tar.x = x;
	Path.pos_tar.y = y;
}

static void PosTarSet(void)
{

	switch (Task.TaskBody[Task.TaskRunIndex].type)
	{
		case 'A':
		case 'w':
			PosTarSetVal(PLACE_A);
			break;

		case 'B':
		case 'x':
			PosTarSetVal(PLACE_B);
			break;

		case 'C':
		case 'y':
			PosTarSetVal(PLACE_C);
			break;

		case 'D':
		case 'z':
			PosTarSetVal(PLACE_D);
			break;
		
		case 'a':
		case 'b':
		case 'c':
			PosTarSetVal(PLACE_abc);
			break;

		case 'd':
		case 'e':
		case 'f':
			PosTarSetVal(PLACE_def);
			break;
	}
}

static void PosTarSetBack(void)
{
	PosTarSetVal(BFS_START);
}

static uint8_t PosCp(void)
{
	if(Path.pos_tar.x == Path.pos.x && Path.pos_tar.y == Path.pos.y) return 1;
	else return 0;
}

static uint8_t RunDir(void)
{
	if (Path.pos.x != Path.path[Path.path_index].x)
	{
		if (Path.path[Path.path_index].x - Path.pos.x > 0)return RIGHT;
		else return LEFT;
	}
	else
	{
		if (Path.path[Path.path_index].y - Path.pos.y > 0)return FRONT;
		else return BACK;
	}
}

static void TaskStateClr(void)
{
	Task.TaskIndex = 0;
	Task.TaskRunIndex = 0;
	Task.ItemNum = 0;
	Task.PlaceNum = 0;
	Task.TaskFlag = None;
	Task.StartCorrected = 0;

}

static void TaskStateUpdate(void)
{
	Task.TaskBody[Task.TaskRunIndex].type = 0;
	Task.TaskBody[Task.TaskRunIndex].val = 0;
	Task.TaskRunIndex++;
}

static void PathStateClr(void)
{
	Path.path_len = 0;
	Path.path_index = 1;
	Path.pos_tar.x = BFS_START_X;
	Path.pos_tar.y = BFS_START_Y;
}

static uint8_t StartZoneGetPoint(Point *start_point)
{
	if (start_point == 0)
		return 0;

	switch (Task.StartZone)
	{
		case StartZone1:
			start_point->x = START_ZONE1_BFS_X;
			start_point->y = START_ZONE1_BFS_Y;
			return 1;

		case StartZone2:
			start_point->x = START_ZONE2_BFS_X;
			start_point->y = START_ZONE2_BFS_Y;
			return 1;

		default:
			return 0;
	}
}

static uint8_t BfsResetStartZone(void)
{
	Point start_point;

	if (!StartZoneGetPoint(&start_point))
		return 0;

	Path.pos.x = start_point.x;
	Path.pos.y = start_point.y;
	Path.pos_tar.x = start_point.x;
	Path.pos_tar.y = start_point.y;
	Path.path_len = 0;
	Path.path_index = 1;
	return 1;
}

/*
 * 将物理启动校正后的车辆位置登记为BFS入口。
 * 启停区不在地图网格内，若只清空路径而保留旧Path.pos，第一次寻路会从旧坐标开始。
 */
static void BfsResetStart(void)
{
	Path.pos.x = BFS_START_X;
	Path.pos.y = BFS_START_Y;
	Path.pos_tar.x = BFS_START_X;
	Path.pos_tar.y = BFS_START_Y;
	Path.path_len = 0;
	Path.path_index = 1;
}
static void PathStateUpdate(void)
{
	// ��ֵ��ǰ���ڵ�λ��
	Path.pos.x = Path.path[Path.path_index].x;
	Path.pos.y = Path.path[Path.path_index].y;
	// ˳�����path
	Path.path[Path.path_index].x = 0;
	Path.path[Path.path_index].y = 0;
	// �ж��Ƿ��������Ǿ͸�������
	if(Path.path_index < Path.path_len - 1)Path.path_index++;
}

static void CalPathWithTask(void)
{
	CalPath(Path.pos.x,Path.pos.y,Path.pos_tar.x,Path.pos_tar.y);
}

static uint16_t GetPlate(uint8_t boolbool)
{
   for (uint8_t i = 0; i < PLATE_NUM; i++)
   {
		// ��ʾ���λ��û�зŶ���
		if (plate_state[i] == boolbool)
		{
			plate_state[i] = !boolbool;
			switch (i)
			{
				case 0:
					return JOINT1_PLATE1;
					break;

				case 1:
					return JOINT1_PLATE2;
					break;

				case 2:
					return JOINT1_PLATE3;
					break;
			}
		}
   }
   return 0;
}

static void BackCheck(void)
{
	/* 正式流程未启用；若IMU未到位，直接中止本次回查动作。 */
	if (!Rotation(0))
		return;
	/*
	 * === 旧版：后退8cm（已弃用）===
	 * 问题：偏移量对不上新起点(0,4)
	RunMove(FRONT,8,0);
	 * === 新版：FRONT 27 + LEFT 25，启动校准 ===
	 */
	RunMove(FRONT,25,0);
	RunMove(LEFT,22,0);
	Servo_Plate_To_Move(JOINT1_PLATE1);
	SetCameraTask(CAM_PLACE_HOME);
	WaitPlace(CAM_PLACE_HOME);
	Servo_Move_To_Plate(JOINT1_PLATE1);
}

static void PickUp(void)
{
	switch (Task.TaskBody[Task.TaskRunIndex].type)
	{
		case 'A':
			Rotation(yaw - 90);  // CW90°
			Servo_Plate_To_Move(JOINT1_PLATE1);
			SetCameraTask(CAM_PLACE_A);
			WaitPlace(CAM_PLACE_A);

			SetCameraTask(CAM_ITEM_A_1);
			WaitPlace(CAM_ITEM_A_1);

			Catch(GetPlate(0));

			SetCameraTask(CAM_PLACE_A);	
			WaitPlace(CAM_PLACE_A);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
		case 'B':
			Rotation(yaw - 90);  // CW90°
			Servo_Plate_To_Move(JOINT1_PLATE1);
			SetCameraTask(CAM_PLACE_B);
			WaitPlace(CAM_PLACE_B);

			SetCameraTask(CAM_ITEM_B_1);
			WaitPlace(CAM_ITEM_B_1);

			Catch(GetPlate(0));

			SetCameraTask(CAM_PLACE_B);	
			WaitPlace(CAM_PLACE_B);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
		case 'C':
			Rotation(yaw - 90);  // CW90°
			Servo_Plate_To_Move(JOINT1_PLATE1);
			SetCameraTask(CAM_PLACE_C);
			WaitPlace(CAM_PLACE_C);
		
			SetCameraTask(CAM_ITEM_C_1);
			WaitPlace(CAM_ITEM_C_1);

			Catch(GetPlate(0));

			SetCameraTask(CAM_PLACE_C);	
			WaitPlace(CAM_PLACE_C);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
		case 'D':
			Rotation(yaw - 90);  // CW90°
			Servo_Plate_To_Move(JOINT1_PLATE1);
			SetCameraTask(CAM_PLACE_D);
			WaitPlace(CAM_PLACE_D);

			SetCameraTask(CAM_ITEM_D_1);
			WaitPlace(CAM_ITEM_D_1);

			Catch(GetPlate(0));
			RunMove(FRONT,10,0);

			SetCameraTask(CAM_PLACE_D);	
			WaitPlace(CAM_PLACE_D);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
		
		case 'a':
			Rotation(0);
			Servo_Plate_To_Move(JOINT1_PLATE1);

			SetCameraTask(CAM_PLACE_abc);
			WaitPlace(CAM_PLACE_abc);

			RunMove(LEFT,INTERVAL_DIS,0);
			RunMove(FRONT,10,0);

			SetCameraTask(CAM_PLACE_a);
			WaitPlace(CAM_PLACE_a);

			Place(GetPlate(1));

			RunMove(BACK,10,0);
			RunMove(RIGHT,INTERVAL_DIS,0);

			SetCameraTask(CAM_PLACE_abc);	
			WaitPlace(CAM_PLACE_abc);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
		case 'b':
			Rotation(0);
			Servo_Plate_To_Move(JOINT1_PLATE1);

			SetCameraTask(CAM_PLACE_abc);
			WaitPlace(CAM_PLACE_abc);

			RunMove(FRONT,10,0);

			SetCameraTask(CAM_PLACE_b);
			WaitPlace(CAM_PLACE_b);

			Place(GetPlate(1));

			RunMove(BACK,10,0);

			SetCameraTask(CAM_PLACE_abc);	
			WaitPlace(CAM_PLACE_abc);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
		case 'c':
			Rotation(0);
			Servo_Plate_To_Move(JOINT1_PLATE1);

			SetCameraTask(CAM_PLACE_abc);
			WaitPlace(CAM_PLACE_abc);

			RunMove(RIGHT,INTERVAL_DIS,0);
			RunMove(FRONT,10,0);

			SetCameraTask(CAM_PLACE_c);
			WaitPlace(CAM_PLACE_c);

			Place(GetPlate(1));

			RunMove(BACK,10,0);
			RunMove(LEFT,INTERVAL_DIS,0);

			SetCameraTask(CAM_PLACE_abc);	
			WaitPlace(CAM_PLACE_abc);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;

		case 'd':
			Rotation(-90);
			RunMove(FRONT,10,0);

			SetCameraTask(CAM_PLACE_def);
			WaitPlace(CAM_PLACE_def);

			RunMove(LEFT,INTERVAL_DIS,0);		

			SetCameraTask(CAM_PLACE_d);
			WaitPlace(CAM_PLACE_d);

			Place(GetPlate(1));

			Servo_Move_To_Plate(JOINT1_PLATE1);

			RunMove(RIGHT,INTERVAL_DIS,0);

			break;
		case 'e':
			Rotation(-90);
			RunMove(FRONT,10,0);

			SetCameraTask(CAM_PLACE_def);
			WaitPlace(CAM_PLACE_def);

			SetCameraTask(CAM_PLACE_e);
			WaitPlace(CAM_PLACE_e);

			Place(GetPlate(1));

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
		case 'f':
			Rotation(-90);
			RunMove(FRONT,10,0);

			SetCameraTask(CAM_PLACE_def);
			WaitPlace(CAM_PLACE_def);

			RunMove(RIGHT,INTERVAL_DIS,0);	

			SetCameraTask(CAM_PLACE_f);
			WaitPlace(CAM_PLACE_f);

			Place(GetPlate(1));

			Servo_Move_To_Plate(JOINT1_PLATE1);

			RunMove(LEFT,INTERVAL_DIS,0);
			break;

		case 'w':
		case 'x':
		case 'y':
		case 'z':
			Rotation(yaw - 90);  // CW90°
			Servo_Plate_To_Move(JOINT1_PLATE1);
			SetCameraTask(CAM_PLACE_A);
			WaitPlace(CAM_PLACE_A);

			reset_detector(&detector);
			SetCameraTask(CAM_YOLO);
			WaitYolo();

			SetCameraTask(CAM_PLACE_A);	
			WaitPlace(CAM_PLACE_A);

			Servo_Move_To_Plate(JOINT1_PLATE1);
			break;
	}
}


/*
 * === 旧版：默认BFS一定成功（已停用）===
 * 问题：坐标越界或无路径时仍会读取Path.path[1]，可能误驱动车轮或死循环。
 * === 新版：旧实现完整保留在条件编译块中 ===
 */
#if 0
uint8_t RunMap(void)
{
	if (!PosCp())	// 未到目标
	{
		CalPathWithTask();
		while (!PosCp())
		{
			uint8_t dir = RunDir();
			uint16_t combo = 1;	// 同方向连续格数

			/* 往后看：同方向就并起来，一次走多格 */
			while (Path.path_index + combo < Path.path_len)
			{
				Point *a = &Path.path[Path.path_index + combo - 1];
				Point *b = &Path.path[Path.path_index + combo];
				uint8_t next_dir;
				if (b->x != a->x)
					next_dir = (b->x > a->x) ? RIGHT : LEFT;
				else
					next_dir = (b->y > a->y) ? FRONT : BACK;

				if (next_dir == dir) combo++;
				else break;
			}

			TurnIMU(DirToYaw(dir) - yaw);
			HAL_Delay(100);
			RunMove(dir, UNIT_DIS * combo, 1);
			for (uint16_t i = 0; i < combo; i++)
				PathStateUpdate();
		}
		PathStateClr();
	}
	return 1;
}
#endif

static uint8_t GridPointValid(Point point)
{
	return point.x >= 0 && point.x < ROWS &&
	       point.y >= 0 && point.y < COLS;
}

static uint8_t PathResultValid(void)
{
	if (Path.path_len < 2 || Path.path_len > ROWS * COLS)
		return 0;
	if (Path.path_index != 1)
		return 0;
	if (Path.path[0].x != Path.pos.x || Path.path[0].y != Path.pos.y)
		return 0;
	if (Path.path[Path.path_len - 1].x != Path.pos_tar.x ||
	    Path.path[Path.path_len - 1].y != Path.pos_tar.y)
		return 0;

	for (int i = 0; i < Path.path_len; i++)
	{
		if (!GridPointValid(Path.path[i]))
			return 0;
		if (i > 0)
		{
			int dx = Path.path[i].x - Path.path[i - 1].x;
			int dy = Path.path[i].y - Path.path[i - 1].y;
			if (my_abs(dx) + my_abs(dy) != 1)
				return 0;
		}
	}

	return 1;
}

/* 每两个41cm网格强制停车并用IMU复核一次绝对朝向。 */
#define MAP_HEADING_CHECK_CM (UNIT_DIS * 2)

#if 0
/*
 * === 后续版：保持工位朝向时转换车体方向（已回退）===
 * 地图：+X向左，+Y向下；车辆yaw=0时车头朝+Y、机械臂朝-Y。
 */
static uint8_t MapDirToBodyDir(uint8_t map_dir, short hold_yaw, uint8_t *body_dir)
{
	int32_t normalized_yaw;

	if (body_dir == NULL)
		return 0;

	normalized_yaw = NormalizeAngleError((int32_t)hold_yaw);
	switch (normalized_yaw)
	{
		case 0:
			switch (map_dir)
			{
				case RIGHT: *body_dir = LEFT;  return 1; /* 地图+X */
				case LEFT:  *body_dir = RIGHT; return 1; /* 地图-X */
				case FRONT: *body_dir = FRONT; return 1; /* 地图+Y */
				case BACK:  *body_dir = BACK;  return 1; /* 地图-Y */
			}
			break;

		case -90:
			switch (map_dir)
			{
				case RIGHT: *body_dir = FRONT; return 1;
				case LEFT:  *body_dir = BACK;  return 1;
				case FRONT: *body_dir = RIGHT; return 1;
				case BACK:  *body_dir = LEFT;  return 1;
			}
			break;

		case 90:
			switch (map_dir)
			{
				case RIGHT: *body_dir = BACK;  return 1;
				case LEFT:  *body_dir = FRONT; return 1;
				case FRONT: *body_dir = LEFT;  return 1;
				case BACK:  *body_dir = RIGHT; return 1;
			}
			break;

		case 180:
		case -180:
			switch (map_dir)
			{
				case RIGHT: *body_dir = RIGHT; return 1;
				case LEFT:  *body_dir = LEFT;  return 1;
				case FRONT: *body_dir = BACK;  return 1;
				case BACK:  *body_dir = FRONT; return 1;
			}
			break;
	}

	return 0;
}
#endif

uint8_t RunMap(void)
{
	uint16_t moved_steps = 0;
	/* 后续版拐角计数（已回退）：uint8_t segment_index = 0; */

	if (!GridPointValid(Path.pos) || !GridPointValid(Path.pos_tar))
	{
		RotationStopAll();
		PathStateClr();
		return 0;
	}

	/*
	 * 旧顺序先检查PosCp；若两个损坏坐标恰好相等，会把非法位置误判为到达。
	 * 新顺序先验证坐标，再接受“当前位置等于目标”。
	 */
	if (PosCp())
		return 1;

	CalPathWithTask();
	if (!PathResultValid())
	{
		RotationStopAll();
		PathStateClr();
		return 0;
	}

	while (!PosCp())
	{
		uint8_t dir;
		/* 后续版车体方向换算（已回退）：uint8_t body_dir = FRONT; */
		uint16_t combo = 1;

		if (Path.path_index >= Path.path_len || moved_steps >= ROWS * COLS)
		{
			RotationStopAll();
			PathStateClr();
			return 0;
		}

		dir = RunDir();
		while (Path.path_index + combo < Path.path_len)
		{
			Point *a = &Path.path[Path.path_index + combo - 1];
			Point *b = &Path.path[Path.path_index + combo];
			uint8_t next_dir;

			if (b->x != a->x)
				next_dir = (b->x > a->x) ? RIGHT : LEFT;
			else
				next_dir = (b->y > a->y) ? FRONT : BACK;

			if (next_dir != dir)
				break;
			combo++;
		}

		/*
		 * === 上一版：整段路线始终保持离站工位朝向（已停用）===
		 * 问题：粗加工区到暂存区应先保持yaw=180沿+X移动，在路径拐角
		 *       再一次转到暂存区yaw=90；若整段都保持180，到站仍要补转。
		 * === 新版：第一直线不转，进入第二直线前一次对准目标站朝向 ===
		 */
		/*
		 * === 后续版：第二直线前提前对准目标工位（已回退）===
		 * if (CoarseTravelYawValid && segment_index > 0 && !CoarseDestinationYawReady)
		 * {
		 *     if (!Rotation(Task.StationYaw))
		 *     {
		 *         RotationStopAll();
		 *         PathStateClr();
		 *         return 0;
		 *     }
		 *     CoarseTravelYaw = Task.StationYaw;
		 *     CoarseDestinationYawReady = 1;
		 * }
		 */

		/*
		 * === 旧版：每段BFS都强制转到道路固定朝向（已停用）===
		 * 问题：粗加工区已把机械臂一次对准180°，去暂存区时又会转回0°，
		 *       表现为到站后多余旋转。
		 * if (!TurnIMU(DirToYaw(dir) - yaw))
		 * {
		 *     RotationStopAll();
		 *     PathStateClr();
		 *     return 0;
		 * }
		 * === 新版：已有工位朝向时只换算车体平移方向，不旋转车身 ===
		 */
		/*
		 * === 后续版：保持工位朝向并换算车体平移方向（已回退）===
		 * if (CoarseTravelYawValid)
		 * {
		 *     if (!MapDirToBodyDir(dir, CoarseTravelYaw, &body_dir))
		 *     {
		 *         RotationStopAll();
		 *         PathStateClr();
		 *         return 0;
		 *     }
		 * }
		 * else if (!TurnIMU(DirToYaw(dir) - yaw))
		 * {
		 *     RotationStopAll();
		 *     PathStateClr();
		 *     return 0;
		 * }
		 * === 恢复版：每段道路开始前对准该道路方向 ===
		 */
		if (!TurnIMU(DirToYaw(dir) - yaw))
		{
			RotationStopAll();
			PathStateClr();
			return 0;
		}
		HAL_Delay(100);
		/*
		 * === 旧版：同方向所有网格一次走完（已停用）===
		 * 问题：连续路线可能超过82cm，中途没有强制读取新IMU并修正姿态。
		 * {
		 *     uint16_t map_distance = UNIT_DIS * combo;
		 *     RunMove(dir, 5, 1);
		 *     HAL_Delay(1500);
		 *     RunMove(dir, map_distance - 5, 1);
		 * }
		 * for (uint16_t i = 0; i < combo; i++)
		 * {
		 *     PathStateUpdate();
		 *     moved_steps++;
		 * }
		 * === 新版：最多两格(82cm)一段，整段结束后强制闭环校正 ===
		 */
		{
			uint16_t remaining_steps = combo;

			while (remaining_steps > 0)
			{
				uint16_t chunk_steps = (remaining_steps > 2) ? 2 : remaining_steps;
				uint16_t map_distance = UNIT_DIS * chunk_steps;

				/* 保留方向修改后的5cm观察窗口，再完成本段剩余距离。 */
				if (map_distance > 5)
				{
					/*
					 * 后续版工位朝向平移（已回退）：
					 * if (CoarseTravelYawValid) RunMove(body_dir, 5, 0);
					 * else RunMove(dir, 5, 1);
					 */
					RunMove(dir, 5, 1);
					HAL_Delay(1500);
					/*
					 * 后续版工位朝向平移（已回退）：
					 * if (CoarseTravelYawValid) RunMove(body_dir, map_distance - 5, 0);
					 * else RunMove(dir, map_distance - 5, 1);
					 */
					RunMove(dir, map_distance - 5, 1);
				}
				else
				{
					/*
					 * 后续版工位朝向平移（已回退）：
					 * if (CoarseTravelYawValid) RunMove(body_dir, map_distance, 0);
					 * else RunMove(dir, map_distance, 1);
					 */
					RunMove(dir, map_distance, 1);
				}

				for (uint16_t i = 0; i < chunk_steps; i++)
				{
					PathStateUpdate();
					moved_steps++;
				}
				remaining_steps -= chunk_steps;

				/* 走满82cm必须用新IMU帧校正到本段绝对目标朝向。 */
				if (map_distance == MAP_HEADING_CHECK_CM)
				{
					/* 后续版保持工位朝向（已回退）：short heading_target = CoarseTravelYawValid ? CoarseTravelYaw : DirToYaw(dir); */
					short heading_target = DirToYaw(dir);
					if (!Rotation(heading_target))
					{
						RotationStopAll();
						PathStateClr();
						return 0;
					}
					HAL_Delay(100);
				}
			}
		}
		/* 后续版拐角计数（已回退）：segment_index++; */
	}

	PathStateClr();
	return 1;
}

/*
 * === 粗定位参数 ===
 * 校正距离尚未完成实测，默认不发送平移指令，只校正车头 yaw=0。
 * 实测后只需修改下面两个参数，不改状态机。
 */
#define START_CORRECT_X_CM 0
#define START_CORRECT_Y_CM 0

/*
 * === 旧版：所有启停区共用零距离校正（已停用）===
 * 问题：无法表达S1/S2不同的物理入口动作。
 */
#if 0
static void StartCorrection(void)
{
	if (START_CORRECT_X_CM > 0)
		RunMove(RIGHT, START_CORRECT_X_CM, 0);
	if (START_CORRECT_Y_CM > 0)
		RunMove(BACK, START_CORRECT_Y_CM, 0);
}
#endif

static uint8_t StartCorrection(void)
{
	/*
	 * 后续版工位朝向状态清理（已回退）：
	 * CoarseTravelYawValid = 0;
	 * CoarseTravelYaw = 0;
	 * CoarseDestinationYawReady = 0;
	 */

	if (!Imu_Wait_Ready(1500))
		return 0;

	/* IMU未到位时禁止继续执行启动平移。 */
	if (!Rotation(0))
		return 0;

	switch (Task.StartZone)
	{
		case StartZone1:
			/*
			 * === 旧版：S1启动先向RIGHT横移（已停用）===
			 * 问题：S1和S2的车尾均朝地图-Y，实车RIGHT对应地图-X，
			 *       会从右边界开出场地。
			 * RunMove(RIGHT, 25, 0);
			 * HAL_Delay(600);
			 * RunMove(FRONT, 25, 0);
			 * HAL_Delay(600);
			 * === 新版：先沿地图+X进入场地，再沿地图+Y到(0,0)入口 ===
			 */
			RunMove(LEFT, 5, 0);    /* 先用5cm确认沿地图+X驶入 */
			HAL_Delay(1500);
			RunMove(LEFT, 20, 0);   /* 补足横移25cm */
			HAL_Delay(600);
			RunMove(FRONT, 25, 0);  /* 沿地图+Y进入S1的BFS入口 */
			HAL_Delay(600);
			break;

		case StartZone2:
			/*
			 * === 旧版：S2启动先向RIGHT横移（已停用）===
			 * 问题：实车摆放为机械臂朝地图-Y，RIGHT实际对应地图-X，
			 *       第一段动作会直接开出场地，无法登记BFS入口(0,4)。
			 * RunMove(RIGHT, 25, 0);
			 * HAL_Delay(600);
			 * RunMove(BACK, 25, 0);
			 * HAL_Delay(600);
			 * === 新版：先向地图+X进入车道，再向地图-Y到S2的BFS入口 ===
			 */
			/*
			 * === 上一版：LEFT一次走完25cm（已停用）===
			 * 问题：方向刚修正时没有短距离观察窗口，若现场坐标判断仍有偏差，
			 *       车辆会连续走完整段距离。
			 * RunMove(LEFT, 25, 0);
			 * HAL_Delay(600);
			 * === 新版：先走5cm验证方向，再补足剩余20cm ===
			 */
			RunMove(LEFT, 5, 0);    /* 关键验证：应沿地图+X驶入场内 */
			HAL_Delay(1500);        /* 留出观察和紧急停止窗口 */
			RunMove(LEFT, 20, 0);   /* 方向正确后补足，总横移仍为25cm */
			HAL_Delay(600);
			RunMove(BACK, 25, 0);   /* 机械臂朝-Y时，BACK对应地图-Y */
			HAL_Delay(600);
			break;

		default:
			return 0;
	}

	return BfsResetStartZone();
}

static uint8_t StopCorrection(void)
{
	if (!Imu_Wait_Ready(1500))
		return 0;

	/* IMU未到位时禁止继续执行返回启停区的平移。 */
	if (!Rotation(0))
		return 0;

	switch (Task.StartZone)
	{
		case StartZone1:
			/*
			 * === 旧版：S1回程使用BACK后LEFT（已停用）===
			 * 问题：启动首段改为LEFT后，回程末段必须使用RIGHT才能原路返回。
			 * RunMove(BACK, 25, 0);
			 * HAL_Delay(600);
			 * RunMove(LEFT, 25, 0);
			 * HAL_Delay(600);
			 * === 新版：严格执行S1启动动作的逆序反向 ===
			 */
			RunMove(BACK, 25, 0);   /* 撤销启动时的FRONT */
			HAL_Delay(600);
			RunMove(RIGHT, 25, 0);  /* 撤销启动时的LEFT，回到S1 */
			HAL_Delay(600);
			break;

		case StartZone2:
			/*
			 * === 旧版：S2回程使用FRONT后LEFT（已停用）===
			 * 问题：启动第一段改为LEFT后，回程末段必须使用RIGHT才能原路返回。
			 * RunMove(FRONT, 25, 0);
			 * HAL_Delay(600);
			 * RunMove(LEFT, 25, 0);
			 * HAL_Delay(600);
			 * === 新版：严格执行启动动作的逆序反向 ===
			 */
			RunMove(FRONT, 25, 0);  /* 撤销启动时的BACK */
			HAL_Delay(600);
			RunMove(RIGHT, 25, 0);  /* 撤销启动时的LEFT，回到S2区域 */
			HAL_Delay(600);
			break;

		default:
			return 0;
	}

	RotationStopAll();
	return 1;
}

/*
 * === 旧版：粗定位只导航到单个工位并等待机械臂（已停用）===
 * 问题：无法自动完成二维码区、三个工位和所选出口的整套测试路线。
 * === 新版：旧实现完整保留在条件编译块中 ===
 */
#if 0
static uint8_t CoarseTargetSet(void)
{
	switch (Task.CoarseTarget)
	{
		case CoarseTargetMaterial:
			Path.pos_tar.x = MATERIAL_COARSE_X;
			Path.pos_tar.y = MATERIAL_COARSE_Y;
			Task.StationYaw = MATERIAL_STATION_YAW;
			return 1;

		case CoarseTargetRough:
			Path.pos_tar.x = ROUGH_COARSE_X;
			Path.pos_tar.y = ROUGH_COARSE_Y;
			Task.StationYaw = ROUGH_STATION_YAW;
			return 1;

		case CoarseTargetStorage:
			Path.pos_tar.x = STORAGE_COARSE_X;
			Path.pos_tar.y = STORAGE_COARSE_Y;
			Task.StationYaw = STORAGE_STATION_YAW;
			return 1;

		default:
			return 0;
	}
}

void CoarseNavigateTo(uint8_t target)
{
	if (target < CoarseTargetMaterial || target > CoarseTargetStorage)
		return;

	/* This entry is for travel between stations already inside the BFS map. */
	Task.TaskIndex = 0;
	Task.TaskRunIndex = 0;
	Task.ItemNum = 0;
	Task.PlaceNum = 0;
	Task.CoarseTarget = target;
	Task.CoarseStage = CoarseBfsNavigate;
	Task.TaskFlag = Start;
}

void CoarseProcesing(void)
{
	if (Task.TaskFlag != Start || Task.CoarseStage == CoarseIdle)
		return;

	switch ((CoarseStage)Task.CoarseStage)
	{
		case CoarseStartCorrect:
			if (!StartCorrection())
			{
				RotationStopAll();
				Task.CoarseFaultCode = CoarseFaultImuTimeout;
				Task.CoarseStage = CoarseFault;
				Task.TaskFlag = None;
				break;
			}
			Task.CoarseStage = CoarseBfsNavigate;
			break;

		case CoarseBfsNavigate:
			/*
			 * === 旧版：只允许导航到原料区（已停用）===
			 * if (Task.CoarseTarget != CoarseTargetMaterial)
			 * {
			 *     Task.CoarseStage = CoarseIdle;
			 *     return;
			 * }
			 * Path.pos_tar.x = MATERIAL_COARSE_X;
			 * Path.pos_tar.y = MATERIAL_COARSE_Y;
			 * === 新版：按目标选择工位坐标和机械臂朝向 ===
			 */
			if (!CoarseTargetSet())
			{
				Task.CoarseStage = CoarseIdle;
				return;
			}

			RunMap();
			Task.CoarseStage = CoarseStationAlign;
			break;

		case CoarseStationAlign:
			/* Material zone is at the -Y side of (2,0); arm/camera face -Y at yaw=0. */
			Rotation(Task.StationYaw);
			Stop();
			Task.CoarseStage = CoarseWaitArm;
			break;

		case CoarseWaitArm:
			/* Hardware is not connected yet; hold position for later arm integration. */
			break;

		default:
			Task.CoarseStage = CoarseIdle;
			break;
	}
}
#endif

static uint8_t CoarseTargetSet(void)
{
	Point return_point;

	switch (Task.CoarseTarget)
	{
		case CoarseTargetQr:
			Path.pos_tar.x = QR_SCAN_X;
			Path.pos_tar.y = QR_SCAN_Y;
			Task.StationYaw = QR_SCAN_YAW;
			return 1;

		case CoarseTargetMaterial:
			Path.pos_tar.x = MATERIAL_COARSE_X;
			Path.pos_tar.y = MATERIAL_COARSE_Y;
			Task.StationYaw = MATERIAL_STATION_YAW;
			return 1;

		case CoarseTargetRough:
			Path.pos_tar.x = ROUGH_COARSE_X;
			Path.pos_tar.y = ROUGH_COARSE_Y;
			Task.StationYaw = ROUGH_STATION_YAW;
			return 1;

		case CoarseTargetStorage:
			Path.pos_tar.x = STORAGE_COARSE_X;
			Path.pos_tar.y = STORAGE_COARSE_Y;
			Task.StationYaw = STORAGE_STATION_YAW;
			return 1;

		case CoarseTargetReturn:
			if (!StartZoneGetPoint(&return_point))
				return 0;
			Path.pos_tar.x = return_point.x;
			Path.pos_tar.y = return_point.y;
			return 1;

		default:
			return 0;
	}
}

static uint8_t CoarseAdvanceTarget(void)
{
	/* 后续版目标朝向状态清理（已回退）：CoarseDestinationYawReady = 0; */

	switch (Task.CoarseTarget)
	{
		case CoarseTargetQr:
			Task.CoarseTarget = CoarseTargetMaterial;
			return 1;

		case CoarseTargetMaterial:
			Task.CoarseTarget = CoarseTargetRough;
			return 1;

		case CoarseTargetRough:
			Task.CoarseTarget = CoarseTargetStorage;
			return 1;

		case CoarseTargetStorage:
			Task.CoarseTarget = CoarseTargetReturn;
			return 1;

		default:
			return 0;
	}
}

static void CoarseEnterFault(uint8_t fault_code)
{
	RotationStopAll();
	/*
	 * 后续版工位朝向状态清理（已回退）：
	 * CoarseTravelYawValid = 0;
	 * CoarseDestinationYawReady = 0;
	 */
	Task.CoarseFaultCode = fault_code;
	Task.CoarseStage = CoarseFault;
	Task.TaskFlag = None;
}

static void CoarseFinishTest(void)
{
	/*
	 * 后续版工位朝向状态清理（已回退）：
	 * CoarseTravelYawValid = 0;
	 * CoarseDestinationYawReady = 0;
	 */
	Task.CoarseTestPassed = 1;
	Task.CoarseFaultCode = CoarseFaultNone;
	Task.CoarseTarget = CoarseTargetNone;
	Task.CoarseMode = CoarseModeNone;
	Task.StartCorrected = 0;
	Task.StartZone = StartZoneNone;
	Task.CoarseStage = CoarseFinished;
	Task.TaskFlag = None;
}

void CoarseNavigateTo(uint8_t target)
{
	if (target < CoarseTargetMaterial || target > CoarseTargetReturn)
		return;

	Task.TaskIndex = 0;
	Task.TaskRunIndex = 0;
	Task.ItemNum = 0;
	Task.PlaceNum = 0;
	Task.CoarseTarget = target;
	/* 后续版目标朝向状态清理（已回退）：CoarseDestinationYawReady = 0; */
	Task.CoarseStage = CoarseBfsNavigate;
	Task.TaskFlag = Start;
}

void CoarseProcesing(void)
{
	if (Task.TaskFlag != Start || Task.CoarseStage == CoarseIdle)
		return;

	switch ((CoarseStage)Task.CoarseStage)
	{
		case CoarseStartCorrect:
			if (!StartCorrection())
			{
				CoarseEnterFault(CoarseFaultImuTimeout);
				break;
			}
			Task.CoarseStage = CoarseBfsNavigate;
			break;

		case CoarseBfsNavigate:
			if (!CoarseTargetSet())
			{
				CoarseEnterFault(CoarseFaultState);
				break;
			}

			if (!RunMap())
			{
				CoarseEnterFault(CoarseFaultBfsPath);
				break;
			}

			if (Task.CoarseTarget == CoarseTargetReturn)
			{
				if (!StopCorrection())
				{
					CoarseEnterFault(CoarseFaultImuTimeout);
					break;
				}
				CoarseFinishTest();
				break;
			}

			Task.CoarseStage = CoarseStationAlign;
			break;

		case CoarseStationAlign:
			/*
			 * === 后续版：BFS拐角对准后到站跳过Rotation（已回退）===
			 * if (!CoarseDestinationYawReady)
			 * {
			 *     if (!Rotation(Task.StationYaw))
			 *     {
			 *         CoarseEnterFault(CoarseFaultImuTimeout);
			 *         break;
			 *     }
			 * }
			 * CoarseTravelYaw = Task.StationYaw;
			 * CoarseTravelYawValid = 1;
			 * CoarseDestinationYawReady = 1;
			 * === 恢复版：到站后按工位角度对准 ===
			 */
			if (!Rotation(Task.StationYaw))
			{
				CoarseEnterFault(CoarseFaultImuTimeout);
				break;
			}
			Stop();
			Task.CoarseStage = CoarseStationDwell;
			break;

		case CoarseStationDwell:
			HAL_Delay(COARSE_TEST_DWELL_MS);
			if (!CoarseAdvanceTarget())
			{
				CoarseEnterFault(CoarseFaultState);
				break;
			}
			Task.CoarseStage = CoarseBfsNavigate;
			break;

		case CoarseFault:
		case CoarseFinished:
			Task.TaskFlag = None;
			break;

		case CoarseWaitArm:
		default:
			CoarseEnterFault(CoarseFaultState);
			break;
	}
}

void RunTask(void)
{
	PickUp();
	TaskStateUpdate();
}

/*
 * === 新版 Procesing：区分搬运/巡检/回航 ===
 * 搬运模式(ItemNum>0)：导航到地点 + 抓取物品
 * 巡检模式(ItemNum==0)：只导航到地点，不抓取
 */
static void InspectTask(void)
{
	/* 到站后按站点定朝向，使机械臂对准工作区 */
	switch (Task.TaskBody[Task.TaskRunIndex].type)
	{
		case 'a': case 'b': case 'c':
			Rotation(-180);  break;	// abc区：车头朝-y，机械臂朝+y

		case 'd': case 'e': case 'f':
			Rotation(90);    break;	// def区：车头朝-x，机械臂朝+x
	}
	TaskStateUpdate();
}

void Procesing(void)
{	
	if(Task.TaskFlag == Start)
	{
		if (Task.ItemNum + Task.PlaceNum != 0)
		{
			if(Task.TaskRunIndex < Task.ItemNum + Task.PlaceNum)
			{
					/*
					 * === 旧版：进入正式任务后直接执行两段校正（已停用）===
					 * 问题：没有等待新鲜IMU数据、没有先回正，且BACK段后立即进入BFS，
					 * 车辆仍可能处于减速/残余运动状态。
					 * if (Task.TaskRunIndex == 0)
					 * {
					 *     RunMove(RIGHT, 25, 0);
					 *     HAL_Delay(600);
					 *     RunMove(BACK, 25, 0);
					 * }
					 * === 新版：IMU就绪→车头回正→分段校正→停稳→登记BFS入口 ===
					 */
					if (Task.TaskRunIndex == 0 && Task.StartCorrected == 0)
					{
						if (!Imu_Wait_Ready(1500))
						{
							/* IMU未就绪时保持TaskRunIndex=0，下一轮继续等待。 */
							HAL_Delay(100);
							return;
						}

						Rotation(0);
						RunMove(RIGHT, 25, 0);
						HAL_Delay(600);
						RunMove(BACK, 25, 0);
						HAL_Delay(600);
						BfsResetStart();
						Task.StartCorrected = 1;
					}
				PosTarSet();
				RunMap();
				if (Task.ItemNum > 0)
					RunTask();       // 搬运：导航 + 抓取
				else
					InspectTask();   // 巡检：仅导航
			}
			else	// 全部完成，返回起点
			{
				PosTarSetBack();
				RunMap();
				/*
				 * === 旧版：BFS结束后立即执行停止校正（已弃用）===
				 * 问题：未确认IMU是否已更新，可能使用旧yaw执行Rotation(0)。
				 * BackCheck();
				 * === 新版：停止校正前重新等待连续3帧有效IMU数据 ===
				 */
				if (!Imu_Wait_Ready(1500))
				{
					HAL_Delay(100);
					return;
				}
				BackCheck();
				TaskStateClr();
			}
		}
	}
}
