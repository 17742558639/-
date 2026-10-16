#include "imu.h"
#include "usart.h"

/*
	HWT101 5v
	TX: PB10  RX:PB11  DMA1 1

	0x55	0x53	0x00	0x00	0x00	0x00	YawL	YawH	VL	VH	SUM

	0x00	
	0x00	
	0x00	
	0x00	
	YawL	
	YawH	
	VL	
	VH	
	SUM	  SUM=0x55+0x53+YawH+YawL+VL+VH
*/


__IO uint8_t ImuCmd[IMU_CMD] = {0};	
__IO uint8_t ImuCount = 0;	

extern DMA_HandleTypeDef hdma_usart3_rx;

__IO short yaw = 0,yaw_last = 0,yaw_now = 0;	
__IO short yaw_desired = 0;						
int yaw_change_flag = 0;	

#define IMU_READY_FRAME_COUNT 3

static __IO uint8_t ImuValidFrameCount = 0;
static __IO uint8_t ImuDataReady = 0;



const uint8_t clear_arr[5] = {0xFF,0xAA,0x76,0x00,0x00};


void Imu_Set_Zero(void)
{
	HAL_UART_Transmit(&huart3,clear_arr,5,50);
	HAL_Delay(3000);
}


void Imu_Init(void)
{											
	Imu_Set_Zero();
	ImuValidFrameCount = 0;
	ImuDataReady = 0;
	
	__HAL_UART_CLEAR_IDLEFLAG(&huart3); 						
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 				
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)ImuCmd, IMU_CMD); 	
}


void Imu_Get(void)
{
	uint8_t frame_valid = 0;

	if(ImuCount == IMU_CMD)
	{
		if(ImuCmd[0] == 0x55 && ImuCmd[1] == 0x53)
		{
			uint8_t Checksum = ImuCmd[9] + ImuCmd[8] + ImuCmd[7] + ImuCmd[6] + ImuCmd[0] + ImuCmd[1];	
			if(Checksum == ImuCmd[10])
			{
				yaw_now = (float)((int16_t)(ImuCmd[7]<<8 | ImuCmd[6])) / 32768.0f * 180.0f;
				
				if(yaw_now - yaw_last>180) 
				{
					yaw_change_flag--;
				}else if(yaw_now - yaw_last<-180) 
				{
					yaw_change_flag++;
				}
				yaw_last = yaw_now;
				yaw = yaw_now+ yaw_change_flag*360;		
				frame_valid = 1;
			}
		}
	}

	if (frame_valid)
	{
		if (ImuValidFrameCount < IMU_READY_FRAME_COUNT)
			ImuValidFrameCount++;
		if (ImuValidFrameCount >= IMU_READY_FRAME_COUNT)
			ImuDataReady = 1;
	}
	else
	{
		ImuValidFrameCount = 0;
		ImuDataReady = 0;
	}
}

/*
 * 等待启动校正使用的新鲜IMU数据。
 * 每次调用都清除旧状态，必须重新连续收到3帧校验正确的数据才返回成功。
 */
uint8_t Imu_Wait_Ready(uint32_t timeout_ms)
{
	uint32_t tick_start;
	uint32_t primask;

	primask = __get_PRIMASK();
	__disable_irq();
	ImuValidFrameCount = 0;
	ImuDataReady = 0;
	if (primask == 0U)
		__enable_irq();

	tick_start = HAL_GetTick();
	while (!ImuDataReady)
	{
		if ((HAL_GetTick() - tick_start) >= timeout_ms)
			return 0;
		HAL_Delay(5);
	}

	return 1;
}

void Imu_Idle_Func(void)
{
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3); 		

		HAL_UART_DMAStop(&huart3); 								
		 
		/*
		 * === 旧版：由循环DMA剩余计数推导本次帧长（已停用）===
		 * 问题：USART3 RX 使用 DMA_CIRCULAR；一帧11字节收满后NDTR会自动回装，
		 * 此时用 IMU_CMD-NDTR 可能得到0，完整帧永远不会交给Imu_Get()。
		 * ImuCount = IMU_CMD - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		 * === 新版：HWT101固定11字节帧 ===
		 * 空闲中断只在一帧结束后的总线空闲期处理；DMA已停止后，当前缓冲区按
		 * 固定11字节交给解析器。帧头和校验和仍由Imu_Get()严格检查，异常帧不会
		 * 增加就绪计数。
		 */
		ImuCount = IMU_CMD;
		
		Imu_Get();	
		
		HAL_UART_Receive_DMA(&huart3, (uint8_t *)ImuCmd, IMU_CMD);
	}
}

