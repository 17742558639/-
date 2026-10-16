# 智能物流搬运机器人 — 开发规则

## 项目信息

- **芯片**：STM32F407VETx
- **开发环境**：Keil MDK（`MDK-ARM/test1.uvprojx`）
- **底层库**：STM32CubeMX HAL
- **底盘**：麦克纳姆轮 + 4 个 Emm_V5 闭环步进电机（USART1）
- **传感器**：HWT101 IMU（USART3）、MaixCam Pro 摄像头（USART2）
- **执行器**：4 关节机械臂舵机（TIM1 PWM）、HMI 串口屏（USART6）

## 目录结构

```
0504/
├── Core/          ← 不要改（CubeMX 生成）
├── Drivers/       ← 不要改（CMSIS + HAL 库）
├── Models/        ← ★ 只改这里
│   ├── Inc/       ← 头文件
│   └── Src/       ← 源文件
└── MDK-ARM/       ← Keil 工程文件
```

## 编译

```
"C:\Keil_v5\UV4\UV4.exe" -b "test1.uvprojx" -j0 -o "build_log.txt"
```

工程路径：`C:\Users\m'm'm\OneDrive\桌面\finish\0504\MDK-ARM\`

## 硬件约定（已确认，不要改）

### 电机地址
```
      前
  ③(左前) | ①(右前)
  --------+--------
  ④(左后) | ②(右后)
      后

地址: 1=右前, 2=右后, 3=左前, 4=右后
顺时针 = yaw 减小
```

### 陀螺仪（IMU）
```
yaw 是 CCW+（逆时针 yaw 变大）
顺时针转 = Rotation(yaw - 90)
dir 映射 = error>0 → dir=1(CCW)，error<0 → dir=0(CW)
```

### 串口分配
| 串口 | 连接 | 波特率 |
|---|---|---|
| USART1 | 4×Emm_V5 电机 | 115200 |
| USART2 | MaixCam 摄像头 | 115200 |
| USART3 | HWT101 IMU | 115200 |
| USART6 | HMI 串口屏 | 9600 |

## NEVER

- 修改 Core/ 和 Drivers/ 文件夹
- 删除旧代码（注释保留，见下方规则）
- 改电机地址映射
- 改 IMU 方向约定
- 不讲电机地址1和3 对调、2和4对调

## ALWAYS

- 改完代码立即编译验证
- 新函数先在 `Move_Init()` 测试路径里跑通，再接入 RunMap
- 用中文解释代码
- 解释术语时打比方

## 代码修改规则

### 只注释，不删除

修改已有函数时，旧代码用 `/* */` 注释保留，新代码写在下方：

```c
/*
 * === 旧版：开环掐表走（已弃用）===
 * 问题：靠时间估算距离，容易偏
void Old_Function(void)
{
    // 旧逻辑...
}
 * === 新版：位置模式闭环 ===
 * 改为电机自己数编码器判断到位
 */
void Old_Function(void)
{
    // 新逻辑...
}
```

### 新功能先测试

`Move_Init()` 里的硬编码路径是测试区，新函数先在这里跑通，确认没问题再接入 `RunMap()` / `Procesing()`。

## 当前状态

| 功能 | 函数 | 备注 |
|---|---|---|
| 直行 | `RunMove(DIR, DIST, 0)` | 位置模式，IMU 走完才检查 |
| 自转 | `Rotation(yaw - 90)` | IMU 全程闭环 ✅ |
| 直行+IMU | `RunMoveWithIMU(DIR, DIST)` | 速度模式，卡轮待解决 |
| PID | `imu_pid` P=-4, D=-3, 死区±0.5°, 限幅±200 | 已调好 |
| 舵机 | `Servo_Init()` | 已注释，待接入 |
| 摄像头 | `Camera_Init()` | 已注释，待接入 | 
| 最终微调 | `Rotation` 主循环退出后误差≥1°补一刀 | 加上就好不用管 |
| 导航时间 | 只有直行跟转弯 |
