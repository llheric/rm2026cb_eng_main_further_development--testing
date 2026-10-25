# DJI 电机驱动模块文档

## 概述

DJI 电机驱动模块提供了对 DJI 系列电机（M2006、M3508、GM6020）的统一控制接口。该模块基于 CAN 总线通信，支持多电机的分组管理和并发控制。

**文件位置：**
- 头文件：`Components/Inc/dji_motor.h`
- 源文件：`Components/Src/dji_motor.c`

---

## 支持的电机类型

| 电机型号 | 类型枚举 | 用途 | 电流范围 |
|---------|---------|------|---------|
| M2006 | `DJI_MOTOR_M2006` | 小功率电机，常用于供弹机构 | [-10000, 10000] → [-10A, 10A] |
| M3508 | `DJI_MOTOR_M3508` | 中功率电机，常用于摩擦轮、底盘 | [-16384, 16384] → [-20A, 20A] |
| GM6020 | `DJI_MOTOR_GM6020` | 云台电机，高精度位置控制 | [-16384, 16384] → [-3A, 3A] |

---

## 关键数据结构

### 1. DJIMotor - 电机状态数据

存储单个电机的实时反馈数据：

```c
typedef struct {
    int32_t  totalAngle;    // 电机累计转角（编码器刻度）
    uint16_t lastAngle;     // 上一次角度 [0, 8191]
    uint16_t angle;         // 当前角度 [0, 8191]
    int16_t  speed;         // 转速 (RPM)
    int16_t  current;       // 实际电流
    int8_t   temperature;   // 温度 (°C)
} DJIMotor;
```

**字段说明：**
- `totalAngle`：累计旋转角度，单位为编码器刻度（8192 刻度/圈），支持多圈计数
- `angle`：当前编码器角度值，范围 [0, 8191] 映射到 [0°, 360°)
- `speed`：电机转速，单位 RPM（转/分钟）
- `current`：电机实际电流反馈值，不同电机范围不同
- `temperature`：电机温度，单位摄氏度

### 2. DJIMotorConfig - 电机配置

用于在初始化时配置电机参数：

```c
typedef struct {
    uint8_t          escId;      // 物理 ID (1-4)
    DJIMotorType     type;       // 电机类型
    DJIMotorBus      bus;        // CAN 总线
    DJIMotorCtrlMode ctrlMode;   // 控制模式
} DJIMotorConfig;
```

### 3. DJIMotorHandle - 电机句柄

电机的完整句柄，包含配置和运行时数据：

```c
typedef struct {
    DJIMotorType     type;       // 电机类型
    DJIMotorBus      bus;        // CAN 总线
    DJIMotorCtrlMode ctrlMode;   // 控制模式
    
    uint8_t  escId;              // 物理 ID (1-4)
    uint16_t cmdStdId;           // 命令标准 ID
    uint16_t fbkStdId;           // 反馈标准 ID
    uint8_t  arrayIndex;         // 在电机数组中的索引
    
    DJIMotor *motor;             // 指向电机数据的指针
    int16_t  *current;           // 指向控制电流的指针
    
    uint8_t txGroupIndex;        // 发送组索引
    uint8_t txGroupSlot;         // 在发送组中的槽位
} DJIMotorHandle;
```

### 4. DJIMotorTxGroup - 发送分组

用于优化 CAN 总线通信，将同一总线、同一命令 ID 的电机分组发送：

```c
typedef struct {
    CAN_HandleTypeDef *hcan;     // CAN 外设句柄
    uint16_t cmdStdId;           // 命令标准 ID
    DJIMotorHandle *hmotors[4];  // 最多 4 个电机
    uint8_t motorCount;          // 电机数量
} DJIMotorTxGroup;
```

---

## CAN 通信协议

### CAN ID 分配规则

#### M2006 和 M3508
- **反馈 ID**：`0x200 + escId`（物理 ID 1-4）
- **命令 ID**：`0x200`（控制 ID 1-4）

#### GM6020
- **反馈 ID**：`0x204 + escId`（物理 ID 1-4）
- **命令 ID**：`0x1FE`（控制 ID 1-4）

### 反馈数据格式（8 字节）

| 字节 | 字段 | 描述 |
|-----|------|------|
| 0-1 | Angle | 编码器角度 [0, 8191] |
| 2-3 | Speed | 转速 (RPM，有符号) |
| 4-5 | Current | 实际电流反馈（有符号） |
| 6 | Temperature | 温度 (°C) |
| 7 | Reserved | 保留字节 |

### 命令数据格式（8 字节）

每个命令帧可以同时控制 4 个电机：

| 字节 | 字段 | 描述 |
|-----|------|------|
| 0-1 | Motor 1 Current | 电机 1 目标电流（高字节在前） |
| 2-3 | Motor 2 Current | 电机 2 目标电流 |
| 4-5 | Motor 3 Current | 电机 3 目标电流 |
| 6-7 | Motor 4 Current | 电机 4 目标电流 |

---

## API 参考

### 初始化函数

#### `void DJIMotor_Init(void)`

初始化 DJI 电机驱动模块。

**功能：**
1. 清零所有电机数据
2. 根据配置构建电机句柄
3. 构建发送分组以优化通信

**调用时机：** 系统启动时调用一次

**示例：**
```c
// 在 main.c 或初始化函数中
DJIMotor_Init();
```

---

### 数据解包函数

#### `void DJIMotor_ESCDataUnpack(uint8_t const *data, DJIMotor *motor)`

解析电调返回的 CAN 数据包，更新电机状态。

**参数：**
- `data`：8 字节 CAN 数据
- `motor`：目标电机结构体指针

**功能：**
- 解析角度、速度、电流、温度
- 计算角度增量，更新累计角度 `totalAngle`
- 自动处理编码器过零点和反向旋转

**调用时机：** CAN 接收中断回调中

**示例：**
```c
// 在 CAN 接收回调中
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
    
    // 根据 StdId 找到对应的电机
    uint8_t motorId = rxHeader.StdId - 0x200;
    if (hcan == &hcan1) {
        DJIMotor_ESCDataUnpack(rxData, &CAN1_Motors[motorId]);
    }
}
```

---

### 控制发送函数

#### `void DJIMotor_SendAll(void)`

向所有已配置的电机发送控制指令。

**功能：**
- 遍历所有发送组
- 将每组电机的目标电流打包成 CAN 数据帧
- 通过 CAN 总线发送

**调用时机：** 控制周期中（通常 1kHz）

**示例：**
```c
// 在控制任务中
void Control_Task(void) {
    while(1) {
        // 1. 更新控制算法，设置目标电流
        CAN1_MotorCurrents[1] = pid_calculate(...);
        CAN1_MotorCurrents[2] = pid_calculate(...);
        
        // 2. 发送所有电机控制指令
        DJIMotor_SendAll();
        
        osDelay(1); // 1ms 控制周期
    }
}
```

---

## 使用指南

### 1. 配置电机

在 `dji_motor.c` 中修改 `motorConfigs` 数组：

```c
static const DJIMotorConfig motorConfigs[] = {
    // escId, 类型, CAN 总线, 控制模式
    { .escId = 1, .type = DJI_MOTOR_M3508,  .bus = DJI_MOTOR_BUS_CAN1, 
      .ctrlMode = DJI_MOTOR_CTRL_CURRENT}, // 摩擦轮 1
    { .escId = 2, .type = DJI_MOTOR_M3508,  .bus = DJI_MOTOR_BUS_CAN1, 
      .ctrlMode = DJI_MOTOR_CTRL_CURRENT}, // 摩擦轮 2
    { .escId = 1, .type = DJI_MOTOR_GM6020, .bus = DJI_MOTOR_BUS_CAN1, 
      .ctrlMode = DJI_MOTOR_CTRL_CURRENT}, // Pitch 轴
    { .escId = 2, .type = DJI_MOTOR_GM6020, .bus = DJI_MOTOR_BUS_CAN2, 
      .ctrlMode = DJI_MOTOR_CTRL_CURRENT}, // Yaw 轴
    { .escId = 1, .type = DJI_MOTOR_M2006,  .bus = DJI_MOTOR_BUS_CAN2, 
      .ctrlMode = DJI_MOTOR_CTRL_CURRENT}, // 供弹电机
};
```

**注意事项：**
- ⚠️ **物理 ID 限制**：只能使用 1-4，禁止使用 5-8
- 同一 CAN 总线、同一类型电机的 ID 不能重复
- 配置后无需手动修改其他代码，驱动会自动分组

### 2. 初始化流程

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    // 初始化外设
    MX_CAN1_Init();
    MX_CAN2_Init();
    
    // 初始化 DJI 电机驱动
    DJIMotor_Init();
    
    // 启动 CAN
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    
    // 启动操作系统或主循环
    osKernelStart();
}
```

### 3. 读取电机数据

```c
// 访问 CAN1 上的电机 1（M3508 或 M2006）
DJIMotor *motor1 = &CAN1_Motors[1];
int16_t speed = motor1->speed;        // 当前转速
uint16_t angle = motor1->angle;       // 当前角度 [0, 8191]
int32_t totalAngle = motor1->totalAngle; // 累计角度
int8_t temp = motor1->temperature;    // 温度

// 访问 CAN1 上的云台电机 1 (GM6020, 映射到索引 5)
DJIMotor *pitch = &CAN1_Motors[5];
```

**索引映射规则：**
- M2006/M3508：物理 ID 1-4 → 数组索引 1-4
- GM6020：物理 ID 1-4 → 数组索引 5-8

### 4. 控制电机

```c
// 设置目标电流
CAN1_MotorCurrents[1] = 5000;   // M3508 电机 1，约 6.1A
CAN1_MotorCurrents[2] = -3000;  // M3508 电机 2，反向约 3.7A
CAN1_MotorCurrents[5] = 10000;  // GM6020 Pitch，约 1.8A

// 发送控制指令
DJIMotor_SendAll();
```

### 5. 完整控制示例

```c
void Gimbal_Control_Task(void) {
    // PID 控制器
    PID_TypeDef pitch_pid, yaw_pid;
    PID_Init(&pitch_pid, ...);
    PID_Init(&yaw_pid, ...);
    
    while(1) {
        // 获取当前角度
        float pitch_angle = CAN1_Motors[5].totalAngle * 360.0f / 8192.0f;
        float yaw_angle = CAN2_Motors[6].totalAngle * 360.0f / 8192.0f;
        
        // PID 计算
        int16_t pitch_current = PID_Calc(&pitch_pid, target_pitch, pitch_angle);
        int16_t yaw_current = PID_Calc(&yaw_pid, target_yaw, yaw_angle);
        
        // 设置电流
        CAN1_MotorCurrents[5] = pitch_current; // Pitch (GM6020, escId=1)
        CAN2_MotorCurrents[6] = yaw_current;   // Yaw (GM6020, escId=2)
        
        // 发送
        DJIMotor_SendAll();
        
        osDelay(1); // 1kHz 控制频率
    }
}
```

---

## 内部实现机制

### 1. 句柄构建 (`DJIMotor_BuildHandle`)

根据配置为每个电机构建句柄，包括：
- 计算 CAN ID（命令和反馈）
- 确定数组索引
- 关联数据结构指针

### 2. 发送分组 (`DJIMotor_BuildTxGroups`)

将电机按以下规则分组：
- **相同 CAN 总线**
- **相同命令 ID**
- **物理 ID 连续**（1-4）

每组最多 4 个电机，可以通过一个 CAN 帧同时控制。

**优点：**
- 减少 CAN 总线占用
- 提高通信效率
- 自动处理电机分组

### 3. 角度累计算法

```c
uint16_t deltaAngle = (uint16_t)(motor->angle - motor->lastAngle) & 0x1fff;
motor->totalAngle += deltaAngle;
if (deltaAngle & 0x1000) {
    motor->totalAngle -= MOTOR_ENCODER_RESOLUTION;
}
```

**原理：**
- 使用模运算处理 [0, 8191] 的循环
- 通过位运算判断旋转方向（正转/反转）
- 支持多圈累计角度

---

## 全局变量

```c
extern DJIMotor CAN1_Motors[13];      // CAN1 电机数据数组
extern DJIMotor CAN2_Motors[13];      // CAN2 电机数据数组
extern int16_t CAN1_MotorCurrents[13]; // CAN1 目标电流数组
extern int16_t CAN2_MotorCurrents[13]; // CAN2 目标电流数组
```

**索引说明：**
- 索引 0：未使用
- 索引 1-4：M2006/M3508 电机（物理 ID 1-4）
- 索引 5-8：GM6020 电机（物理 ID 1-4）
- 索引 9-12：保留

---

## 常见问题

### Q1: 为什么只能使用物理 ID 1-4？

**A:** 本驱动为了简化实现，只支持物理 ID 1-4。如需使用 5-8，需要修改：
- 命令 ID 映射逻辑（0x200 → 0x1FF，0x1FE → 0x2FF）
- 数组索引计算
- 分组逻辑

### Q2: 如何确定电机索引？

**A:** 使用以下公式：
```c
// M2006/M3508
arrayIndex = escId; // 1-4 → 1-4

// GM6020
arrayIndex = escId + 4; // 1-4 → 5-8
```

### Q3: 电流值如何转换为实际电流？

**A:** 不同电机转换系数不同：
```c
// M3508
float current_A = current_value * 20.0f / 16384.0f;

// M2006
float current_A = current_value * 10.0f / 10000.0f;

// GM6020
float current_A = current_value * 3.0f / 16384.0f;
```

### Q4: 控制频率应该设置为多少？

**A:** TBD

### Q5: 如何调试通信问题？

**A:** TBD

---

## 依赖项

- `can_driver.h`：CAN 驱动抽象层
- `stm32f4xx_hal_can.h`：STM32 HAL CAN 驱动
- `string.h`：内存操作（memset）
