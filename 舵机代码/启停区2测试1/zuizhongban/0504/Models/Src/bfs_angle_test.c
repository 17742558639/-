/* ============================================================
   bfs_angle_test.c — 角度最短路径测试
   目的：验证 TurnIMU() 和 Rotation() 在所有跨圈情况下
         是否走 ≤180° 的最短方向

   用法：PC 上用 gcc 编译运行，不依赖硬件
     gcc bfs_angle_test.c -o test && ./test

   测试用例：
     1. yaw= 350, target=   0  → 应转 -10°(CW)  或 +350°(CCW)
     2. yaw= 540, target=   0  → 应转 ±180°
     3. yaw=-270, target=  90  → 应转 0°（等价角度）
     4. yaw= 720, target= -90  → 应转 -90°(CW)  或 +270°(CCW)
     5. yaw= 180, target=-180  → 应转 0°
     6. yaw= 450, target=   0  → 应转 -90°(CW)
   ============================================================ */

#include <stdio.h>
#include <stdlib.h>

/* 模拟 STM32 的 short 类型 */
typedef short int16_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

#define my_abs(x) ((x) < 0 ? -(x) : (x))

/* ============================================================
   当前 Rotation() 的错误计算方式（无归一化）
   返回：总共需要的旋转次数（模拟迭代直到误差<3°）
   ============================================================ */
static int Rotation_TotalDegrees(int16_t yaw_current, int16_t angle_desired)
{
    int16_t yaw = yaw_current;
    int16_t target = angle_desired;
    int total_degrees = 0;
    int iterations = 0;
    int stable_cnt = 0;

    while (stable_cnt < 2 && iterations < 50)
    {
        int16_t error = target - yaw;

        if (my_abs(error) > 3)
        {
            stable_cnt = 0;
            uint16_t step;
            if (my_abs(error) > 90)
                step = 90;
            else
                step = (uint16_t)((float)my_abs(error) * 0.8f);
            if (step < 1) step = 1;

            int dir = (error > 0) ? 1 : -1;
            yaw += dir * (int16_t)step;
            total_degrees += step;
        }
        else
        {
            stable_cnt++;
        }
        iterations++;
    }

    /* 最终微调 */
    {
        int16_t error = target - yaw;
        if (my_abs(error) >= 1)
        {
            int dir = (error > 0) ? 1 : -1;
            uint16_t step = (uint16_t)my_abs(error);
            if (step < 1) step = 1;
            yaw += dir * (int16_t)step;
            total_degrees += step;
        }
    }

    return total_degrees;
}

/* ============================================================
   最短路径参考值：把 error 归一化到 [-180, 180]
   ============================================================ */
static int16_t NormalizeError(int16_t error)
{
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;
    return error;
}

/* ============================================================
   TurnIMU 的方式（有归一化）：一步到位算总旋转量
   ============================================================ */
static int TurnIMU_TotalDegrees(int16_t yaw_current, int16_t delta_deg)
{
    int16_t target = yaw_current + delta_deg;
    int16_t error = target - yaw_current;

    /* TurnIMU 的归一化循环 */
    while (error > 180)  { error -= 360; target -= 360; }
    while (error < -180) { error += 360; target += 360; }

    if (my_abs(error) <= 2) return 0;

    /* 第一步：90% */
    int total = (int)((float)my_abs(error) * 0.9f);
    if (total < 2) total = 2;

    /* 模拟旋转后 yaw */
    int16_t yaw_after = yaw_current;
    if (error > 0) yaw_after += total;
    else           yaw_after -= total;

    /* 第二步：微调 */
    error = target - yaw_after;
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;

    if (my_abs(error) >= 1)
        total += my_abs(error) + 1;

    return total;
}

/* ============================================================
   测试主函数
   ============================================================ */
typedef struct {
    int16_t yaw;
    int16_t target;
    const char *desc;
} TestCase;

int main(void)
{
    TestCase cases[] = {
        {  350,    0,  "350°→0°, 应转10°CW或350°CCW(取短=10°)"       },
        {  540,    0,  "540°→0°, yaw=540即物理180°, 应转180°"        },
        { -270,   90,  "-270°→90°, -270=90等价, 应转0°"              },
        {  720,  -90,  "720°→-90°, 720=0等价, 应转90°"               },
        {  180, -180,  "180°→-180°, 应转0°(同一点)"                   },
        {  450,    0,  "450°→0°, 450=90等价, 应转90°"                },
        {    0,   90,  "0°→90°, 基准: 应转90°"                        },
        {   90,   -0,  "90°→0°, 基准: 应转90°"                        },
    };

    int total = sizeof(cases) / sizeof(cases[0]);
    int passed_rotation = 0;
    int passed_turnimu  = 0;

    printf("============================================================\n");
    printf("  角度最短路径测试 — 对比 Rotation() vs TurnIMU()\n");
    printf("============================================================\n\n");

    for (int i = 0; i < total; i++)
    {
        int16_t yaw    = cases[i].yaw;
        int16_t target = cases[i].target;

        /* 最短参考路径 */
        int16_t raw_error = target - yaw;
        int16_t short_error = NormalizeError(raw_error);
        int shortest = my_abs(short_error);

        /* Rotation() 的当前行为 */
        int rot_total = Rotation_TotalDegrees(yaw, target);

        /* TurnIMU() 的当前行为 */
        int16_t delta = target - yaw;  /* TurnIMU 接收 delta */
        int tim_total = TurnIMU_TotalDegrees(yaw, delta);

        int rot_ok = (rot_total <= shortest + 5);  /* 允许5°误差 */
        int tim_ok = (tim_total <= shortest + 5);

        if (rot_ok) passed_rotation++;
        if (tim_ok) passed_turnimu++;

        printf("【测试%d】%s\n", i + 1, cases[i].desc);
        printf("  最短路径应为 ≤ %d°\n", shortest);
        printf("  Rotation(): 实际转了 %d°  → %s\n",
               rot_total, rot_ok ? "✅ 通过" : "❌ 多转！");
        printf("  TurnIMU() : 实际转了 %d°  → %s\n",
               tim_total, tim_ok ? "✅ 通过" : "❌ 多转！");
        printf("\n");
    }

    printf("============================================================\n");
    printf("  结果: Rotation() %d/%d 通过, TurnIMU() %d/%d 通过\n",
           passed_rotation, total, passed_turnimu, total);
    printf("============================================================\n");

    return (passed_rotation == total) ? 0 : 1;
}
