#include "camera.h"

/*
	MaixCam Pro
	TX: PA2  RX:PA3  DMA1 1

*/


__IO uint8_t CameraCmd[CAMERA_CMD] = {0};		
__IO uint8_t CameraCount = 0;	
__IO uint8_t CameraReady = 0;	
uint8_t str[4] = {0xaa,0xbb,0x00,0xcc};

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;

short x = 0,y = 0;
short yolo_classid = 0, yolo_num= 0;
StableDetector detector;
int is_stable = 0;

int update_detector(StableDetector *det, short yolo_classid, short yolo_num);

static void Camera_Get(void)
{
	if (CameraReady == 0)
	{
		if (CameraCmd[0] == 0xaa && CameraCmd[1] == 0xbb && CameraCmd[2] == 0xcc)CameraReady = 1;
	}
	else
	{
		if(str[2] != CAM_YOLO)
		{
			if(CameraCount == CAMERA_CMD)
			{
				if(CameraCmd[0] == 0xaa && CameraCmd[1] == 0xbb && CameraCmd[7] == 0xcc)
				{
					if(CameraCmd[2] == str[2])
					{
						x = (CameraCmd[3] << 8) | CameraCmd[4];
						y = (CameraCmd[5] << 8) | CameraCmd[6];
						memset((uint8_t*)CameraCmd,0,CAMERA_CMD);
					}
				}
			}
		}
		else
		{
			if(CameraCmd[0] == 0xaa && CameraCmd[1] == 0xbb && CameraCmd[5] == 0xcc)
			{
				if(CameraCmd[2] == str[2])
				{
					yolo_classid = CameraCmd[3];
					yolo_num = CameraCmd[4];
					memset((uint8_t*)CameraCmd,0,CAMERA_CMD);
					is_stable = update_detector(&detector, yolo_classid, yolo_num);
				}
			}
		}
		
	}
}

void SetCameraTask(uint8_t which)
{
	str[2] = which;
	for(uint8_t i = 0;i < 5;i++)
	{
		HAL_UART_Transmit_DMA(&huart2,str,4);
		HAL_Delay(100);
	}
}

void WaitPlace(uint8_t which)
{
	int x_err,y_err;
	int pidout_x,pidout_y;
	uint32_t shake_time = 0;

	x = 0;
	y = 0;
	switch (which)
	{
		case CAM_ITEM_A_1:
		case CAM_ITEM_B_1:
		case CAM_ITEM_C_1:
		case CAM_ITEM_D_1:
			x_pid.desired = ITEM_DESIRED_X;
			y_pid.desired = ITEM_DESIRED_Y;
			break;
		case CAM_PLACE_A:
		case CAM_PLACE_B:
		case CAM_PLACE_C:
		case CAM_PLACE_D:
		case CAM_PLACE_a:
		case CAM_PLACE_b:
		case CAM_PLACE_c:
		case CAM_PLACE_d:
		case CAM_PLACE_e:
		case CAM_PLACE_f:
			x_pid.desired = PLATE_DESIRED_X;
			y_pid.desired = PLATE_DESIRED_Y;
			break;
	}
	shake_time = uwTick;
	while(1)
	{
		if(x != 0 && y != 0)
		{
			x_err = x - x_pid.desired;
			y_err = y - y_pid.desired;

			if(my_abs(x_err) > 5 || my_abs(y_err) > 5)	// 超过最大震荡带直接清理
			{
				shake_time = uwTick;
			}

			if(my_abs(x_err) > 2 || my_abs(y_err) > 2)
			{
				pidout_x = (int)pidout(&x_pid,x);	
				pidout_y = (int)pidout(&y_pid,y);	
				PidMove(pidout_x,pidout_y);
				HAL_Delay(10);
			}

			if(uwTick - shake_time > 500)break;
		}
	}
	Stop();
}

// 判断两个结果是否相同（类别和数量都相同）
int is_same_result(YoloResult *a, YoloResult *b) {
    return (a->class_id == b->class_id && a->num == b->num);
}

// 重置检测器
void reset_detector(StableDetector *det) {
    det->candidate.class_id = 0;
    det->candidate.num = 0;
    det->consecutive_count = 0;
    det->stable_result.class_id = 0;
    det->stable_result.num = 0;
    det->is_stable = 0;
    det->start_time = uwTick;
}

// 更新检测器，返回1表示已获得稳定结果，0表示还在收集中
int update_detector(StableDetector *det, short yolo_classid, short yolo_num) {
    #define REQUIRED_CONSECUTIVE 5  // 需要连续10帧完全相同才稳定
    #define TIMEOUT_MS 3000          // 3秒超时
    
    YoloResult current;
    current.class_id = yolo_classid;
    current.num = yolo_num;
    
    // 如果已经稳定，直接返回
    if (det->is_stable) {
        return 1;
    }
    
    // 超时检查
    if (uwTick - det->start_time > TIMEOUT_MS) {
        // 超时，如果有候选结果且连续次数>0，就采用当前候选
        if (det->consecutive_count > 0) {
            det->stable_result = det->candidate;
            det->is_stable = 1;
            return 1;
        }
        // 否则重置重新开始
        reset_detector(det);
        return 0;
    }
    
    // 忽略无效识别（类别为0表示未识别到物体）
    if (yolo_classid == 0) {
        // 数量也归零时，重置候选但不重置计时
        if (yolo_num == 0) {
            det->consecutive_count = 0;
            det->candidate.class_id = 0;
            det->candidate.num = 0;
        }
        return 0;
    }
    
    // 检查是否与当前候选相同
    if (is_same_result(&current, &det->candidate)) {
        // 连续相同，增加计数
        det->consecutive_count++;
        
        // 达到阈值，确认稳定
        if (det->consecutive_count >= REQUIRED_CONSECUTIVE) {
            det->stable_result = det->candidate;
            det->is_stable = 1;
            return 1;
        }
    } else {
        // 不同，更新候选并重置计数
        det->candidate = current;
        det->consecutive_count = 1;
        
        // 可选：如果数量差异太大，完全重置（防止累积干扰）
        if (my_abs(current.num - det->candidate.num) > 5) {
            det->consecutive_count = 0;
        }
    }
    
    return 0;
}

// 获取稳定结果
YoloResult get_stable_result(StableDetector *det) {
    return det->stable_result;
}


YoloResult result;
void WaitYolo(void)
{
	while(!is_stable);
	result = get_stable_result(&detector);
	UpdateShow(result.class_id, result.num);
}

void Camera_Init(void)
{											
	__HAL_UART_CLEAR_IDLEFLAG(&huart2); 						
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); 				
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)CameraCmd, CAMERA_CMD); 	
}

void Camera_Idle_Func(void)
{
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2); 					

		HAL_UART_DMAStop(&huart2); 								
		 
		CameraCount = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);	
		
		Camera_Get();
		
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)CameraCmd, CAMERA_CMD);
	}
}


