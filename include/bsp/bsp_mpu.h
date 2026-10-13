#ifndef __BSP_MPU_H__
#define __BSP_MPU_H__

#include "stm32f1xx_hal.h"

/* ---------------- 器件地址（来源：PS 的 Digital Interface 章节） ---------------- */

/* 7 位器件地址：AD0 接 GND = 0x68，接 VCC = 0x69。
 * ⚠️ AD0 悬空（高阻）时地址不确定，必须明确接死一个电平。
 * 底层 BspI2c_Read/Write 内部已经做了 (addr << 1)，所以这里只填 7 位。 */
#define MPU6050_ADDR                0x68U

/* ---------------- 寄存器地址（来源：RM 的 Register Map 总表） ---------------- */

#define MPU6050_REG_ACCEL_XOUT_H    0x3BU   /* 14 字节突发读的起点：加计 x/y/z + 温度 + 陀螺 x/y/z */
#define MPU6050_REG_GYRO_CFG        0x1BU   /* 陀螺量程：FS_SEL<<3 */
#define MPU6050_REG_ACCEL_CFG       0x1CU   /* 加速度量程：AFS_SEL<<3 */
#define MPU6050_REG_PWR_MGMT_1      0x6BU   /* 电源管理：bit6=SLEEP，默认 1（芯片出厂就在睡） */
#define MPU6050_REG_WHO_AM_I        0x75U   /* 只读身份寄存器，默认 0x68 */

/* WHO_AM_I 期望值：读回来必须等于它，否则不是 6050
 * 参考：0x68 = MPU6050/6000，0x70 = MPU6500，0x71 = MPU9250 */
#define MPU6050_WHO_AM_I_VALUE      0x68U

/* ---------------- 量程与灵敏度（来源：PS 的 Gyro / Accel Specifications 表） ---------------- */

/* ⚠️ 量程和灵敏度必须成对：只改寄存器不改常数（或反过来），数据会整体差一个倍数，且极难发现 */

#define MPU6050_GYRO_RANGE_SEL      0x18U    /* GYRO_CONFIG = FS_SEL(3)<<3 → ±2000°/s */
#define MPU6050_GYRO_LSB_PER_DPS    16.4f    /* ±2000°/s 对应 16.4 LSB/(°/s) */

#define MPU6050_ACCEL_RANGE_SEL     0x00U    /* ACCEL_CONFIG = AFS_SEL(0)<<3 → ±2g */
#define MPU6050_ACCEL_LSB_PER_G     16384.0f /* ±2g 对应 16384 LSB/g */

#define MPU6050_TEMP_LSB_PER_DEG    340.0f   /* 温度灵敏度 */
#define MPU6050_TEMP_OFFSET_DEG     36.53f   /* 温度零点偏置：T = raw/340 + 36.53 */

/* ---------------- 数据结构 ---------------- */

/* 原始值：寄存器里直接读出来的 int16，单位 LSB（未换算、未减零偏） */
typedef struct
{
    int16_t accel[3];   /* x, y, z */
    int16_t gyro[3];    /* x, y, z */
    int16_t temp;
} MpuRaw_t;

/* 物理量：换算后（只换算，零偏统一在 GetData 里处理） */
typedef struct
{
    float accel[3];     /* 单位 g   */
    float gyro[3];      /* 单位 °/s */
    float temp;         /* 单位 ℃  */
} MpuData_t;

/* ---------------- 对外接口 ---------------- */

/* 依赖顺序：必须先在 main 里 BspI2c_Init() 成功后再调本函数 */
HAL_StatusTypeDef BspMpu_Init(void);

/* 读 14 字节 → 大端拆成 7 个 int16；返回失败时 raw 内容无效 */
HAL_StatusTypeDef BspMpu_ReadRaw(MpuRaw_t *raw);

/* raw → g / ℃ / °/s（只换算，不减零偏） */
void BspMpu_RawToData(const MpuRaw_t *raw, MpuData_t *data);

/* 静止校零：采 samples 次求平均，结果存文件内部（static），供 GetData 使用
 * ⚠️ 调用期间车必须静止，否则求出来的不是零偏而是运动量 */
HAL_StatusTypeDef BspMpu_CalibrateGyro(uint16_t samples);

/* 一站式：读 → 换算 → 减零偏。主循环每帧只调这一个 */
HAL_StatusTypeDef BspMpu_GetData(MpuData_t *data);

#endif /* __BSP_MPU_H__ */
