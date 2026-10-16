#include "servo.h"

/*
  ץȡʱ����������11.5cm
*/

extern TIM_HandleTypeDef htim1;
/* 上电前的软件初始记录；Servo_Init()会立即写入实测观察姿态。 */
float g_current_angle[4] = {JOINT1_PLATE2,JOINT2_PLACE_B,JOINT3_PLACE_B,JOINT4_WATCH};
uint8_t plate_state[3] = {0,0,0};


/*
 * === 旧版：初始化后将J4置于闭合值（已弃用）===
 * 问题：实测观察姿态的J4=60，初始化为0会让爪子闭合，
 *       与观察姿态不一致。
 */
/*
void Servo_Init(void)
{
  HAL_Delay(2000);
  Servo_SetAngle(TIM_CHANNEL_1, JOINT1_FRONT);
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_WATCH);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH);
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
*/

/* 新版：上电进入实测观察姿态，J4保持半开。 */
void Servo_Init(void)
{
  HAL_Delay(2000);
  Servo_SetAngle(TIM_CHANNEL_1, JOINT1_FRONT);  // J1=90，正对站点
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_WATCH);  // J2=220，观察高度
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH);  // J3=38，观察高度
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_WATCH);  // J4=60，观察时爪子打开

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/*
 * 兼容task.c中的旧入口Catch_Init()。
 * Servo_Init()已经负责PWM启动和等待，这里不重复延时，只确保四个关节回到
 * 实测观察姿态；which为旧接口参数，当前三个盘子由Catch()单独选择J1。
 */
void Catch_Init(uint32_t which)
{
  (void)which;
  Servo_SetAngle(TIM_CHANNEL_1, JOINT1_FRONT);
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_WATCH);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH);
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_WATCH);
}

/*
 * === 旧版：舵机1按270度、舵机2～4按180度换算（已弃用）===
 * 问题：实际舵机1～3是360度位置型，舵机4是270度位置型。
 * static uint16_t AngleToPulse(float angle,uint32_t which)
 * {
 *   if (which != TIM_CHANNEL_1)
 *   {
 *     if(angle < 0) angle = 0;
 *     if(angle > 180) angle = 180;
 *     return 500 + (uint16_t)(angle * 2000.0f / 180.0f);
 *   }
 *   else
 *   {
 *     if(angle < 0) angle = 0;
 *     if(angle > 270) angle = 270;
 *     return 500 + (uint16_t)(angle * 2000.0f / 270.0f);
 *   }
 * }
 * === 新版：按实际舵机量程换算 ===
 */
static uint16_t AngleToPulse(float angle,uint32_t which)
{
  float max_angle;

  if (which == TIM_CHANNEL_4)
    max_angle = 270.0f;
  else
    max_angle = 360.0f;

  if(angle < 0) angle = 0;
  if(angle > max_angle) angle = max_angle;

  return 500 + (uint16_t)(angle * 2000.0f / max_angle);
}

/* �Ƕ�ת����������һ����λ */
static float GetAngle(uint32_t which) 
{
  float angle = 0.00f;

  switch (which)
  {
    case TIM_CHANNEL_1:
      angle = g_current_angle[0];
      break;

    case TIM_CHANNEL_2:
      angle = g_current_angle[1];
      break;

    case TIM_CHANNEL_3:
      angle = g_current_angle[2];
      break;

    case TIM_CHANNEL_4:
      angle = g_current_angle[3];
      break;
  }
  return angle;
}

/* �Ƕ�ת����������һ����λ */
static void SetAngle(uint32_t which,float angle) 
{
  switch (which)
  {
    case TIM_CHANNEL_1:
      g_current_angle[0] = angle;
      break;

    case TIM_CHANNEL_2:
      g_current_angle[1] = angle;
      break;

    case TIM_CHANNEL_3:
      g_current_angle[2] = angle;
      break;

    case TIM_CHANNEL_4:
      g_current_angle[3] = angle;
      break;
  }
}

/*
 * 按当前缓存同步移动J2/J3到目标姿态。
 * 两个关节每一小步同时更新，避免先动一个关节造成未实测的中间姿态。
 */
static void Servo_Move_Pair_To(int16_t target2, int16_t target3, int step_delay)
{
  int16_t angle2 = (int16_t)GetAngle(TIM_CHANNEL_2);
  int16_t angle3 = (int16_t)GetAngle(TIM_CHANNEL_3);

  while (angle2 != target2 || angle3 != target3)
  {
    if (angle2 < target2)
      angle2 = (angle2 + 2 > target2) ? target2 : angle2 + 2;
    else if (angle2 > target2)
      angle2 = (angle2 - 2 < target2) ? target2 : angle2 - 2;

    if (angle3 < target3)
      angle3 = (angle3 + 2 > target3) ? target3 : angle3 + 2;
    else if (angle3 > target3)
      angle3 = (angle3 - 2 < target3) ? target3 : angle3 - 2;

    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(step_delay);
  }
}

/* ������ýǶȣ����ٶȿ��ƣ�*/
/* which: ��ʱ��ͨ��, angle: Ŀ��Ƕ�, step: �������ȣ�, step_delay: ������ʱ�����룩 */
void Servo_SetAngle_Slow(uint32_t which, float angle, float step, int step_delay) 
{
    // 1. 
   float current_angle = GetAngle(which); 
   uint16_t target_pulse = AngleToPulse(angle, which);
    
    // 2. 
    int direction = (angle > current_angle) ? 1 : -1;
    
    // 3.
    while (fabs(current_angle - angle) > step) 
    { 
        current_angle += step * direction;
        uint16_t pulse = AngleToPulse(current_angle, which);
        __HAL_TIM_SET_COMPARE(&htim1, which, pulse);
        HAL_Delay(step_delay); 
    }
    
    // 4. �ƶ�������Ŀ��Ƕ�
    __HAL_TIM_SET_COMPARE(&htim1, which, target_pulse);
    
    // 5. ���µ�ǰ�Ƕ�״̬
    SetAngle(which,angle);
}


/*
 * === 旧版：预抓取姿态及夹爪动作（已弃用）===
 * 问题：舵机4的渐变循环使用无符号变量配合负数差值，循环不会执行；
 *       舵机4打开为读数增加、闭合为读数减小，旧表达式方向不可靠。
 * === 新版：按实测方向执行舵机4渐变 ===
 */
/*
void Servo_Close_To_Plate(void) 
{

  for(uint8_t i = 0;i < JOINT4_PLATE - JOINT4_OPEN;i += 2)
  {
    Servo_SetAngle(TIM_CHANNEL_4,JOINT4_OPEN + i);
    HAL_Delay(50);
  }
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_PLATE);

	Servo_SetAngle(TIM_CHANNEL_2,30);
  Servo_SetAngle(TIM_CHANNEL_3,18);
  HAL_Delay(150);
	Servo_SetAngle(TIM_CHANNEL_2,35);
  Servo_SetAngle(TIM_CHANNEL_3,25);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,40);
  Servo_SetAngle(TIM_CHANNEL_3,33);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,45);
  Servo_SetAngle(TIM_CHANNEL_3,41);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,50);
  Servo_SetAngle(TIM_CHANNEL_3,50);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,55);
  Servo_SetAngle(TIM_CHANNEL_3,58);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,60);
  Servo_SetAngle(TIM_CHANNEL_3,66);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_PLATE);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_PLATE);
  HAL_Delay(150);

  for(uint8_t i = 0;i < JOINT4_CATCH - JOINT4_PLATE;i += 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4,JOINT4_PLATE + i);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_CATCH);


}
*/


/*
 * === 旧版：靠近车上盘子的动作（已弃用）===
 * 问题：舵机2先从30递增，舵机3先递增到66再减到20，动作中存在反向跳变。
 * === 新版：按姿态起止角单调移动 ===
 */
/*
void Servo_Close_To_Plate(void)
{
  int16_t angle;
  for (angle = JOINT4_OPEN; angle > JOINT4_PLATE; angle -= 2)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle);
    HAL_Delay(50);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_PLATE);
  Servo_SetAngle(TIM_CHANNEL_2,30);
  Servo_SetAngle(TIM_CHANNEL_3,18);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,35);
  Servo_SetAngle(TIM_CHANNEL_3,25);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,40);
  Servo_SetAngle(TIM_CHANNEL_3,33);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,45);
  Servo_SetAngle(TIM_CHANNEL_3,41);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,50);
  Servo_SetAngle(TIM_CHANNEL_3,50);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,55);
  Servo_SetAngle(TIM_CHANNEL_3,58);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,60);
  Servo_SetAngle(TIM_CHANNEL_3,66);
  HAL_Delay(150);
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_PLATE);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_PLATE);
  HAL_Delay(150);
  for (angle = JOINT4_PLATE; angle > JOINT4_CATCH; angle -= 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);
}
*/

/*
 * === 旧版：使用JOINT2_CLOSE_PLATE/JOINT3_CLOSE_PLATE旧终点（已弃用）===
 * 问题：旧终点270/10不是实测B放置姿态，会造成放置位置偏移。
 */
/*
void Servo_Close_To_Plate(void)
{
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_CLOSE_PLATE);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_CLOSE_PLATE);
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);
}
*/

/* B低位放置姿态：J2=360、J3=150，保持J4=0夹紧。 */
void Servo_Close_To_Plate(void)
{
  Servo_Move_Pair_To(JOINT2_PLACE_B, JOINT3_PLACE_B, 60);
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_PLACE_B);
  HAL_Delay(100);
}
/*
 * === 旧版：盘子姿态下降（已弃用）===
 * 问题：固定角度从60/66开始，与上一姿态的宏终点不一致，会产生跳变。
 * === 新版：从盘子姿态单调下降到贴盘姿态 ===
 */
/*
void Servo_Plate_To_Close(void)
{
  Servo_SetAngle(TIM_CHANNEL_2,60);
  Servo_SetAngle(TIM_CHANNEL_3,66);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_2,55);
  Servo_SetAngle(TIM_CHANNEL_3,58);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_2,50);
  Servo_SetAngle(TIM_CHANNEL_3,50);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_2,45);
  Servo_SetAngle(TIM_CHANNEL_3,41);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_2,40);
  Servo_SetAngle(TIM_CHANNEL_3,33);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_2,35);
  Servo_SetAngle(TIM_CHANNEL_3,25);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CLOSE_PLATE);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CLOSE_PLATE);
  HAL_Delay(100);
}
*/

/*
 * 盘子抓取姿态：三个盘子共用J2=280、J3=100、J4=0。
 * J1由Servo_Move_To_Plate()根据盘号选择292/325/360。
 */
void Servo_Plate_To_Close(void)
{
  Servo_Move_Pair_To(JOINT2_PLATE_GRAB, JOINT3_PLATE_GRAB, 70);
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_PLATE_GRAB);
  HAL_Delay(150);
}

/*
 * === 旧版：抓取后夹爪闭合及抬臂动作（已弃用）===
 * 问题：舵机4循环条件为负数，使用uint8_t时循环不会执行。
 * === 新版：夹爪按实测方向闭合 ===
 */
/*
void Servo_Catch_To_Move(void) 
{  
  for(uint8_t i = 0;i < JOINT4_CATCH - JOINT4_OPEN;i += 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4,JOINT4_OPEN + i);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_CATCH);

  for(uint8_t i = 0;i < 102;i +=2)
  {
    if(i <= 86)Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CATCH - i);
    Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CATCH - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_MOVE);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_MOVE);
  HAL_Delay(100);
}
*/

/*
 * === 旧版：抓取后抬臂动作（已弃用）===
 * 问题：抬臂远离地面时，舵机3读数应增加，旧代码却递减。
 * === 新版：舵机2减小、舵机3增加 ===
 */
/*
void Servo_Catch_To_Move(void)
{
  int16_t angle;

  for (angle = JOINT4_OPEN; angle > JOINT4_CATCH; angle -= 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);

  for(uint8_t i = 0;i < 102;i +=2)
  {
    if(i <= 86)Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CATCH - i);
    Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CATCH - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_MOVE);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_MOVE);
  HAL_Delay(100);
}
*/

/*
 * === 旧版：抓取后抬臂动作（已弃用）===
 * 问题：固定 i 的范围与终点宏不一致，舵机3会超过搬运角后再跳回，
 *       舵机2也会先减到过小角度后再跳回搬运角。
 * === 新版：直接按姿态起止角单调移动 ===
 */
/*
void Servo_Catch_To_Move(void)
{
  int16_t angle;
  for (angle = JOINT4_OPEN; angle > JOINT4_CATCH; angle -= 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);
  for(uint8_t i = 0;i < 102;i +=2)
  {
    if(i <= 86)Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CATCH + i);
    Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CATCH - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_MOVE);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_MOVE);
  HAL_Delay(100);
}
*/

/*
 * === 旧版：抓取后J3按增加方向抬臂（已弃用）===
 * 问题：新的实测抓取姿态为J3=150，搬运姿态为J3=110，
 *       抓取到搬运时J3应减小，旧条件会导致J3不渐变而直接跳到终点。
 */
/*
void Servo_Catch_To_Move(void)
{
  int16_t angle2 = JOINT2_CATCH;
  int16_t angle3 = JOINT3_CATCH;
  int16_t angle4;
  for (angle4 = JOINT4_OPEN; angle4 > JOINT4_CATCH; angle4 -= 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle4);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);
  while (angle2 > JOINT2_MOVE || angle3 < JOINT3_MOVE)
  {
    if (angle2 > JOINT2_MOVE) angle2 -= 2;
    if (angle3 < JOINT3_MOVE) angle3 += 2;
    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_MOVE);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_MOVE);
  HAL_Delay(100);
}
*/

/* 抬起物料：正前方抓取J2=360/J3=150 -> 搬运J2=260/J3=110。 */
void Servo_Catch_To_Move(void)
{
  int16_t angle2 = JOINT2_CATCH;
  int16_t angle3 = JOINT3_CATCH;
  /* J4已由Servo_Watch_To_Catch()闭合到0，此处不能先回到70再闭合，否则会松脱物料。 */
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);

  /* 实测搬运方向：J2、J3均从抓取值减小到搬运值。 */
  while (angle2 > JOINT2_MOVE || angle3 > JOINT3_MOVE)
  {
    if (angle2 > JOINT2_MOVE) angle2 -= 2;
    if (angle3 > JOINT3_MOVE) angle3 -= 2;
    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_MOVE);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_MOVE);
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);
  HAL_Delay(100);
}

/*
 * === 旧版：观察/搬运转换只改变舵机3（已弃用）===
 * 问题：舵机2没有从J2=220移动到实测搬运值J2=260，函数不能到达完整姿态。
 */
/*
void Servo_Move_To_Watch(void)
{
  for (uint8_t i = 0; i < JOINT3_MOVE - JOINT3_WATCH; i += 2)
  {
    Servo_SetAngle(TIM_CHANNEL_3, JOINT3_MOVE - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH);
  HAL_Delay(100);
}

void Servo_Watch_To_Move(void)
{
  for (uint8_t i = 0; i < JOINT3_MOVE - JOINT3_WATCH; i += 2)
  {
    Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH + i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_MOVE);
  HAL_Delay(100);
}
*/

/* 新版：观察姿态与搬运姿态的J2、J3都按实测起止值移动。 */
void Servo_Move_To_Watch(void)
{
  int16_t angle2 = JOINT2_MOVE;
  int16_t angle3 = JOINT3_MOVE;

  /* 回观察：J2=260->220，J3=110->38。 */
  while (angle2 > JOINT2_WATCH || angle3 > JOINT3_WATCH)
  {
    if (angle2 > JOINT2_WATCH) angle2 -= 2;
    if (angle3 > JOINT3_WATCH) angle3 -= 2;
    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_WATCH);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH);
  HAL_Delay(100);
}

/* 新版：观察到搬运：J2=220->260，J3=38->110。 */
void Servo_Watch_To_Move(void)
{
  int16_t angle2 = JOINT2_WATCH;
  int16_t angle3 = JOINT3_WATCH;

  while (angle2 < JOINT2_MOVE || angle3 < JOINT3_MOVE)
  {
    if (angle2 < JOINT2_MOVE) angle2 += 2;
    if (angle3 < JOINT3_MOVE) angle3 += 2;
    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_MOVE);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_MOVE);
  HAL_Delay(100);
}

/*
 * A低位摄像头扫描姿态：J1=90、J2=260、J3=60、J4=0。
 * J4先减小到0夹紧物料，再移动J1～J3，避免移动过程中松爪。
 * 此函数只负责舵机，不启动摄像头，也不驱动车体。
 */
void Servo_To_Scan_A(void)
{
  Servo_SetAngle_Slow(TIM_CHANNEL_4, JOINT4_PLACE_B, 5, 70);
  Servo_SetAngle_Slow(TIM_CHANNEL_1, JOINT1_FRONT, 2, 30);
  Servo_Move_Pair_To(JOINT2_SCAN_A, JOINT3_SCAN_A, 60);
  HAL_Delay(100);
}

/*
 * A扫描姿态到B放置姿态：
 * J2=260->360（读数增加，已到舵机2机械极限）；
 * J3=60->150（按实测数值增加）；J4始终保持0夹紧。
 * 到B姿态后必须由上层流程另行执行J4打开，不能在这里提前松爪。
 */
void Servo_Scan_A_To_Place_B(void)
{
  Servo_SetAngle_Slow(TIM_CHANNEL_1, JOINT1_FRONT, 2, 30);
  Servo_Move_Pair_To(JOINT2_PLACE_B, JOINT3_PLACE_B, 60);
  Servo_SetAngle_Slow(TIM_CHANNEL_4, JOINT4_PLACE_B, 5, 70);
  HAL_Delay(200);
}

/* B低位放置后释放物料：J4=0->60，J2/J3保持B姿态不动。 */
void Servo_Release_At_Place(void)
{
  Servo_SetAngle_Slow(TIM_CHANNEL_4, JOINT4_PLATE, 5, 100);
  HAL_Delay(200);
}

/*
 * === 旧版：观察姿态到抓取姿态（已弃用）===
 * 问题：舵机3使用“读数增加”执行弯腰；实测舵机3读数减小才靠近地面。
 * === 新版：弯腰方向按实测方向执行 ===
 */
/*
void Servo_Watch_To_Catch(void) 
{  
  for(uint8_t i = 0;i < JOINT3_CATCH - JOINT3_WATCH;i +=1)
  {
    if(i <= 102)Servo_SetAngle(TIM_CHANNEL_2,JOINT2_WATCH + i);
    Servo_SetAngle(TIM_CHANNEL_3,JOINT3_WATCH + i);
    HAL_Delay(40);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CATCH);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CATCH);
  HAL_Delay(100);
}
*/
/*
 * === 旧版：观察姿态到抓取姿态（已弃用）===
 * 问题：循环上限直接使用 JOINT3_CATCH-JOINT3_WATCH，方向反转后变成负数，
 *       循环完全不执行。
 * === 新版：按两个关节各自的起止角单调移动 ===
 */
/*
void Servo_Watch_To_Catch(void)
{
  int16_t i;
  for (i = 0; i < (JOINT3_CATCH - JOINT3_WATCH); i += 1)
  {
    Servo_SetAngle(TIM_CHANNEL_2, JOINT2_WATCH + i);
    Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH - i);
    HAL_Delay(40);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_CATCH);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_CATCH);
  HAL_Delay(100);
}
*/

/*
 * === 旧版：舵机3按减小方向进入抓取（已弃用）===
 * 问题：正前方实测抓取姿态为J3=150，大于观察姿态J3=38，
 *       旧代码会把J3反向减小到旧的20度。
 */
/*
void Servo_Watch_To_Catch(void)
{
  int16_t angle2 = JOINT2_WATCH;
  int16_t angle3 = JOINT3_WATCH;
  while (angle2 < JOINT2_CATCH || angle3 > JOINT3_CATCH)
  {
    if (angle2 < JOINT2_CATCH) angle2 += 2;
    if (angle3 > JOINT3_CATCH) angle3 -= 2;
    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(40);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_CATCH);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_CATCH);
  HAL_Delay(100);
}
  
*/

/* 观察姿态J2=220/J3=38 -> 正前方抓取J2=360/J3=150。 */
void Servo_Watch_To_Catch(void)
{
  int16_t angle2 = JOINT2_WATCH;
  int16_t angle3 = JOINT3_WATCH;

  /* 本次实测转换中J2、J3均增加；J4最后递减到0夹紧。 */
  while (angle2 < JOINT2_CATCH || angle3 < JOINT3_CATCH)
  {
    if (angle2 < JOINT2_CATCH) angle2 += 2;
    if (angle3 < JOINT3_CATCH) angle3 += 2;
    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(40);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_CATCH);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_CATCH);

  /* 观察J4=60，抓取J4=0：读数减小表示闭合。 */
  for (int16_t angle4 = JOINT4_WATCH; angle4 > JOINT4_CATCH; angle4 -= 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle4);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_CATCH);
  HAL_Delay(100);
}

/*
 * === 旧版：舵机1只支持目标值大于正前方（已弃用）===
 * 问题：which_plate - JOINT1_FRONT 使用uint8_t循环，目标值小于起点时会下溢。
 */
/*
void Servo_Move_To_Plate(uint16_t which_plate)
{
  for (uint8_t i = 0; i < which_plate - JOINT1_FRONT; i += 2)
  {
    Servo_SetAngle(TIM_CHANNEL_1, JOINT1_FRONT + i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_1, which_plate);
  HAL_Delay(100);
}

void Servo_Plate_To_Move(uint16_t which_plate)
{
  for (uint8_t i = 0; i < which_plate - JOINT1_FRONT; i += 2)
  {
    Servo_SetAngle(TIM_CHANNEL_1, which_plate - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_1, JOINT1_FRONT);
  HAL_Delay(100);
}
*/

/*
 * === 旧版：舵机1每2度等待30ms（已弃用）===
 * 问题：从J1=90到第三个盘子J1=360需要约4秒，盘子切换明显过慢。
 * === 新版：舵机1按当前缓存快速渐变 ===
 * 改动：每4度更新一次、间隔15ms；终点和增减方向保持不变。
 */
/*
void Servo_Move_To_Plate(uint16_t which_plate)
{
  int16_t angle = (int16_t)JOINT1_FRONT;
  int16_t target = (int16_t)which_plate;

  if (angle < target)
  {
    for (; angle < target; angle += 2)
    {
      Servo_SetAngle(TIM_CHANNEL_1, (float)angle);
      HAL_Delay(30);
    }
  }
  else if (angle > target)
  {
    for (; angle > target; angle -= 2)
    {
      Servo_SetAngle(TIM_CHANNEL_1, (float)angle);
      HAL_Delay(30);
    }
  }
  Servo_SetAngle(TIM_CHANNEL_1, (float)target);
  HAL_Delay(100);
}
*/

void Servo_Move_To_Plate(uint16_t which_plate)
{
  int16_t angle = (int16_t)GetAngle(TIM_CHANNEL_1);
  int16_t target = (int16_t)which_plate;

  /* J1顺时针读数减小；这里只改变速度，不改变方向和目标角度。 */
  if (angle < target)
  {
    for (; angle < target; angle += 4)
    {
      if (angle > target - 4) angle = target;
      Servo_SetAngle(TIM_CHANNEL_1, (float)angle);
      HAL_Delay(15);
    }
  }
  else if (angle > target)
  {
    for (; angle > target; angle -= 4)
    {
      if (angle < target + 4) angle = target;
      Servo_SetAngle(TIM_CHANNEL_1, (float)angle);
      HAL_Delay(15);
    }
  }

  Servo_SetAngle(TIM_CHANNEL_1, (float)target);
  HAL_Delay(50);
}

/* 新版：从目标盘按数值方向返回J1=90正前方。 */
void Servo_Plate_To_Move(uint16_t which_plate)
{
  int16_t angle = (int16_t)which_plate;
  int16_t target = (int16_t)JOINT1_FRONT;

  if (angle > target)
  {
    for (; angle > target; angle -= 2)
    {
      Servo_SetAngle(TIM_CHANNEL_1, (float)angle);
      HAL_Delay(30);
    }
  }
  else if (angle < target)
  {
    for (; angle < target; angle += 2)
    {
      Servo_SetAngle(TIM_CHANNEL_1, (float)angle);
      HAL_Delay(30);
    }
  }
  Servo_SetAngle(TIM_CHANNEL_1, (float)target);
  HAL_Delay(100);
}

/*
 * === 旧版：贴盘姿态抬起（已弃用）===
 * 问题：只抬舵机3，舵机2仍停留在靠近地面的角度。
 * === 新版：舵机2减小、舵机3增加，协同远离地面 ===
 */
/*
void Servo_Close_Plate_To_Move(void)
{
  for(uint8_t i = 0;i < JOINT3_MOVE - JOINT3_CLOSE_PLATE;i +=2)
  {
    Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CLOSE_PLATE + i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_MOVE);
  HAL_Delay(100);
}
*/

/* 抬起 */
void Servo_Close_Plate_To_Move(void)
{
  /* B低位放置J2=360/J3=150 -> 搬运J2=260/J3=110。 */
  Servo_Move_Pair_To(JOINT2_MOVE, JOINT3_MOVE, 60);
  HAL_Delay(100);
}

/*
 * === 旧版：搬运姿态下降到贴盘姿态（已弃用）===
 * 问题：只降低舵机3，没有让舵机2按靠近地面的方向协同动作。
 * === 新版：舵机2增加、舵机3减小，协同靠近地面 ===
 */
/*
void Servo_Move_To_Close_Plate(void)
{
  for(uint8_t i = 0;i < JOINT3_MOVE - JOINT3_CLOSE_PLATE;i +=2)
  {
    Servo_SetAngle(TIM_CHANNEL_3,JOINT3_MOVE - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CLOSE_PLATE);
  HAL_Delay(100);
}
*/

/* 下降到盘子高度 */
void Servo_Move_To_Close_Plate(void)
{
  /* 搬运J2=260/J3=110 -> B低位放置J2=360/J3=150。 */
  Servo_Move_Pair_To(JOINT2_PLACE_B, JOINT3_PLACE_B, 60);
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_PLACE_B);
  HAL_Delay(100);
}

/*
 * === 旧版：进入盘子姿态（已弃用）===
 * 问题：舵机3使用递增表达式，与靠近地面时读数减小的实测方向相反。
 * === 新版：舵机2递增、舵机3递减 ===
 */
/*
void Servo_Plate_In(void)
{
  for(uint8_t i = 0;i < JOINT2_PLATE - JOINT2_MOVE;i +=5)
  {
    if(i <= 15)Servo_SetAngle(TIM_CHANNEL_3,JOINT3_MOVE + i);
    Servo_SetAngle(TIM_CHANNEL_2,JOINT2_MOVE + i);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_PLATE);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_PLATE);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_PLATE);
}
*/

/* 进入三个盘子的共同抓取姿态：先打开，再下降，最后由Plate_To_Close闭合。 */
void Servo_Plate_In(void)
{
  /* 盘子抓取前J4保持打开，避免下降时夹爪碰撞物料。 */
  Servo_SetAngle_Slow(TIM_CHANNEL_4, JOINT4_PLATE, 5, 70);
  Servo_Move_Pair_To(JOINT2_PLATE_GRAB, JOINT3_PLATE_GRAB, 70);
  HAL_Delay(100);
}

/*
 * === 旧版：放置后回观察姿态（已弃用）===
 * 问题：舵机4打开时使用了递减表达式，且循环条件为负数，实际不会渐变。
 * === 新版：舵机4从闭合位递增到打开位 ===
 */
/*
void Servo_Place_To_Watch(void)
{
  for(uint8_t i = 0;i < JOINT4_CATCH - JOINT4_OPEN;i += 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4,JOINT4_CATCH - i);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_OPEN);

  for(uint8_t i = 0;i < JOINT3_CATCH - JOINT3_WATCH;i +=2)
  {
    if(i <= 102)Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CATCH - i);
    Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CATCH - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_WATCH);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_WATCH);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_OPEN);
}
*/

/*
 * === 旧版：放置后回观察姿态的新实现（已弃用）===
 * 问题：机械臂抬起远离地面时，舵机3读数应增加，旧代码却递减。
 * === 新版：舵机3按实测方向增加 ===
 */
/*
void Servo_Place_To_Watch(void)
{
  int16_t angle;

  for (angle = JOINT4_CATCH; angle < JOINT4_OPEN; angle += 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_OPEN);

  for(uint8_t i = 0;i < JOINT3_CATCH - JOINT3_WATCH;i +=2)
  {
    if(i <= 102)Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CATCH - i);
    Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CATCH - i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_WATCH);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_WATCH);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_OPEN);
}
*/

/*
 * === 旧版：打开夹爪后回观察姿态（已弃用）===
 * 问题：方向宏修正后，JOINT3_CATCH-JOINT3_WATCH为负数，循环不会执行。
 * === 新版：按抓取姿态到观察姿态的起止角单调移动 ===
 */
/*
void Servo_Place_To_Watch(void)
{
  int16_t angle;
  for (angle = JOINT4_CATCH; angle < JOINT4_OPEN; angle += 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_OPEN);
  for(uint8_t i = 0;i < JOINT3_CATCH - JOINT3_WATCH;i +=2)
  {
    if(i <= 102)Servo_SetAngle(TIM_CHANNEL_2,JOINT2_CATCH - i);
    Servo_SetAngle(TIM_CHANNEL_3,JOINT3_CATCH + i);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_WATCH);
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_WATCH);
  HAL_Delay(100);
  Servo_SetAngle(TIM_CHANNEL_4,JOINT4_OPEN);
}
*/

void Servo_Place_To_Watch(void)
{
  int16_t angle2 = JOINT2_CATCH;
  int16_t angle3 = JOINT3_CATCH;
  int16_t angle4;

  /* 夹爪打开：读数递增。 */
  for (angle4 = JOINT4_CATCH; angle4 < JOINT4_OPEN; angle4 += 5)
  {
    Servo_SetAngle(TIM_CHANNEL_4, (float)angle4);
    HAL_Delay(70);
  }
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_OPEN);

  /* 远离地面：舵机2读数减小，舵机3读数增加。 */
  while (angle2 > JOINT2_WATCH || angle3 < JOINT3_WATCH)
  {
    if (angle2 > JOINT2_WATCH) angle2 -= 2;
    if (angle3 < JOINT3_WATCH) angle3 += 2;
    Servo_SetAngle(TIM_CHANNEL_2, (float)angle2);
    Servo_SetAngle(TIM_CHANNEL_3, (float)angle3);
    HAL_Delay(30);
  }
  Servo_SetAngle(TIM_CHANNEL_2, JOINT2_WATCH);
  Servo_SetAngle(TIM_CHANNEL_3, JOINT3_WATCH);
  HAL_Delay(100);
}

/*
 * === 旧版：放置后回搬运姿态（已弃用）===
 * 问题：舵机3使用 JOINT3_PLATE - JOINT3_MOVE 作为 uint8_t 循环上限；
 *       当前目标值为 20 -> 50，差值为负，循环完全不执行。
 * === 新版：舵机2递减、舵机3递增 ===
 */
/*
void Servo_Place_Plate_To_Move(void)
{
  for(uint8_t i = 0;i < JOINT2_PLATE - JOINT2_MOVE;i += 2)
  {
    Servo_SetAngle(TIM_CHANNEL_2,JOINT2_PLATE - i);
    HAL_Delay(50);
  }
  Servo_SetAngle(TIM_CHANNEL_2,JOINT2_MOVE);

  for(uint8_t i = 0;i < JOINT3_PLATE - JOINT3_MOVE;i += 2)
  {
    Servo_SetAngle(TIM_CHANNEL_3,JOINT3_PLATE - i);
    HAL_Delay(50);
  }
  Servo_SetAngle(TIM_CHANNEL_3,JOINT3_MOVE);
}
*/

//回搬运姿态
void Servo_Place_Plate_To_Move(void)
{
  /* B低位放置J2=360/J3=150 -> 搬运J2=260/J3=110。 */
  Servo_Move_Pair_To(JOINT2_MOVE, JOINT3_MOVE, 60);
}

/*
 * === 旧版：只写PWM，不更新当前角度缓存（已弃用）===
 * 问题：Servo_SetAngle_Slow()读取g_current_angle时可能从旧角度开始渐变。
 */
/*
void Servo_SetAngle(uint32_t which, float angle)
{
  uint16_t pulse = AngleToPulse(angle, which);
  __HAL_TIM_SET_COMPARE(&htim1, which, pulse);
}
*/

/* 新版：写PWM的同时同步缓存，保证后续慢速动作起点正确。 */
void Servo_SetAngle(uint32_t which, float angle)
{
  uint16_t pulse = AngleToPulse(angle, which);
  __HAL_TIM_SET_COMPARE(&htim1, which, pulse);
  SetAngle(which, angle);
}


/*
 * === 旧版：先在正前方抓取，再转盘，盘子动作混用旧贴盘终点（已弃用）===
 * 问题：旧流程没有把三个盘子的共同抓取姿态明确分开，且返回后又重复进入正前方抓取。
 */
/*
void Catch(uint32_t which)
{
  Servo_Watch_To_Catch();
  Servo_Catch_To_Move();
  Servo_Move_To_Plate(which);
  Servo_Plate_In();
  Servo_Plate_To_Close();
  Servo_Close_Plate_To_Move();
  Servo_Plate_To_Move(which);
  Servo_Move_To_Watch();
  Servo_SetAngle(TIM_CHANNEL_4, JOINT4_OPEN);
}

void Place(uint32_t which)
{
  Servo_Watch_To_Move();
  Servo_Move_To_Plate(which);
  Servo_Move_To_Close_Plate();
  Servo_Close_To_Plate();
  Servo_Place_Plate_To_Move();
  Servo_Plate_To_Move(which);
  Servo_Move_To_Watch();
  Servo_Watch_To_Catch();
  Servo_Place_To_Watch();
}
*/

/*
 * 从三个盘子之一抓取物料。
 * which只选择J1：292/325/360；J2/J3/J4统一使用280/100/0。
 */
void Catch(uint32_t which)
{
  Servo_Watch_To_Move();              // 先抬到搬运高度
  Servo_Move_To_Plate((uint16_t)which); // J1转到指定盘子
  Servo_Plate_In();                   // J2=280,J3=100，J4打开
  Servo_Plate_To_Close();             // J4=0，夹紧物料
  Servo_Close_Plate_To_Move();        // 回到J2=260,J3=110
  Servo_Plate_To_Move((uint16_t)which); // J1回正前方J1=90
}

/*
 * 在车辆已完成A姿态视觉对准和微调后放置物料。
 * which不再参与放置角度：B姿态实测J1固定为90。
 */
void Place(uint32_t which)
{
  (void)which;
  Servo_Scan_A_To_Place_B();          // A：260/60 -> B：360/150，保持J4=0
  Servo_Release_At_Place();           // B姿态J4=0 -> 60，释放物料
  Servo_Place_Plate_To_Move();        // 抬回J2=260,J3=110
  Servo_Move_To_Watch();               // 回观察姿态
}

