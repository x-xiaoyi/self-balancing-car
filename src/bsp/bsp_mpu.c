#include "bsp/bsp_mpu.h"
#include "bsp/bsp_i2c.h"

/* ---------------- 文件内部私有区（static，不进 .h） ---------------- */

/* 单次 I2C 读写的超时（ms）。正常只需几百 us，
 * 100ms 是"故障时能快速报错退出"的量级；太大=出错时白等。 */
#define MPU_I2C_TIMEOUT     100U

/* 陀螺零偏（单位 °/s）：CalibrateGyro 写入，GetData 每帧减掉。
 * 初值 0 表示"还没校准"——此时减 0 等于没减，不会算出错值，是安全的默认。 */
static float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

/* 写一个寄存器：把 BspI2c_Write 里每次都不变的参数（器件地址、长度、超时）收口 */
static HAL_StatusTypeDef mpu_write_reg(uint8_t reg, uint8_t val)
{
    /* &val：val 是形参，活到本函数返回；BspI2c_Write 在此之前就把这 1 字节发完，
     * 不会留下悬垂指针，所以直接取它的地址当发送缓冲区是安全的。 */
    return BspI2c_Write(MPU6050_ADDR, reg, &val, 1U, MPU_I2C_TIMEOUT);
}

/* 连续读 len 个寄存器：突发读（14 字节）就靠它，一次读完整帧 */
static HAL_StatusTypeDef mpu_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return BspI2c_Read(MPU6050_ADDR, reg, buf, len, MPU_I2C_TIMEOUT);
}

/* 大端拼接（高字节在前）：先转 uint16 再移位，最后转回 int16 保住符号。
 * 若直接用两个 uint8 拼，负值会丢符号 —— 陀螺反向转时你只会读到正数。 */
static int16_t mpu_combine_be(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

/* ---------------- 对外接口 ---------------- */

HAL_StatusTypeDef BspMpu_Init(void)
{
    uint8_t who = 0U;

    /* ① 自检：先确认"在跟谁说话"。
     * 两种失败要分开看：HAL 错=没人应答（接线/供电/地址）；
     * 值不对=有人应答但不是 6050（地址被占 / 芯片型号不对）。 */
    if (mpu_read_reg(MPU6050_REG_WHO_AM_I, &who, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (who != MPU6050_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    /* ② 唤醒：bit6 SLEEP 置 0；低 3 位 CLKSEL=001，选 PLL(X gyro) 做时钟源，
     * 比内部 8MHz RC 振荡器稳。不唤醒的话，后面读到的数据寄存器永远停在复位值。 */
    if (mpu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* ③ 陀螺量程：写入值和 MPU6050_GYRO_LSB_PER_DPS 是一对，改一个必须改另一个 */
    if (mpu_write_reg(MPU6050_REG_GYRO_CFG, MPU6050_GYRO_RANGE_SEL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* ④ 加速度量程：同理和 MPU6050_ACCEL_LSB_PER_G 成对 */
    if (mpu_write_reg(MPU6050_REG_ACCEL_CFG, MPU6050_ACCEL_RANGE_SEL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BspMpu_ReadRaw(MpuRaw_t *raw)
{
    uint8_t buf[14] = {0};

    /* 一次从 ACCEL_XOUT_H 读满 14 字节：加计 6 + 温度 2 + 陀螺 6。
     * 必须一次读完 —— 分几次读的话，6 个轴不是同一时刻的采样，
     * 后面融合时"加速度算的角"和"陀螺积分的角"时间对不上，越算越飘。 */
    if (mpu_read_reg(MPU6050_REG_ACCEL_XOUT_H, buf, 14U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    /* 顺序固定：先加计 XYZ，再温度，最后陀螺 XYZ —— 这是寄存器排布决定的 */
    raw->accel[0] = mpu_combine_be(buf[0],  buf[1]);
    raw->accel[1] = mpu_combine_be(buf[2],  buf[3]);
    raw->accel[2] = mpu_combine_be(buf[4],  buf[5]);
    raw->temp     = mpu_combine_be(buf[6],  buf[7]);
    raw->gyro[0]  = mpu_combine_be(buf[8],  buf[9]);
    raw->gyro[1]  = mpu_combine_be(buf[10], buf[11]);
    raw->gyro[2]  = mpu_combine_be(buf[12], buf[13]);
    return HAL_OK;
}
void BspMpu_RawToData(const MpuRaw_t *raw, MpuData_t *data)
{
    /* 除以灵敏度常数：这些常数必须和 Init 里配的量程成对，否则整体差倍数 */
    data->accel[0] = (float)raw->accel[0] / MPU6050_ACCEL_LSB_PER_G;
    data->accel[1] = (float)raw->accel[1] / MPU6050_ACCEL_LSB_PER_G;
    data->accel[2] = (float)raw->accel[2] / MPU6050_ACCEL_LSB_PER_G;
    data->gyro[0] = (float)raw->gyro[0] / MPU6050_GYRO_LSB_PER_DPS;
    data->gyro[1] = (float)raw->gyro[1] / MPU6050_GYRO_LSB_PER_DPS;
    data->gyro[2] = (float)raw->gyro[2] / MPU6050_GYRO_LSB_PER_DPS;
    /* 温度是"除以灵敏度 + 加零点偏置"，和另外两路换算方式不同 */
    data->temp = (float)raw->temp / MPU6050_TEMP_LSB_PER_DEG + MPU6050_TEMP_OFFSET_DEG;
}
HAL_StatusTypeDef BspMpu_CalibrateGyro(uint16_t samples)
{
    /* 用 int32 累加：samples 次 int16 相加会超出 int16 范围，用 int16 累加必溢出 */
    int32_t sum[3] = {0, 0, 0};
    MpuRaw_t raw = {0};
    /* 除零保护：samples 为 0 时下面会除以 0，属于未定义行为，直接挡掉 */
    if (samples == 0U)
    {
        return HAL_ERROR;
    }
    for (uint16_t i = 0; i < samples; i++)
    {
        if (BspMpu_ReadRaw(&raw) != HAL_OK)
        {
            return HAL_ERROR;
        }
        sum[0] += raw.gyro[0];
        sum[1] += raw.gyro[1];
        sum[2] += raw.gyro[2];
        /* 两次采样之间隔开一点：连续猛读会采到同一批噪声，平均掉的效果变差。
         * ⚠️ HAL_Delay 是阻塞延时，校准期间会卡住整个程序，所以放在开机阶段最合适。 */
        HAL_Delay(2U);
    }
    /* 平均 → 再除灵敏度换成 °/s，存进文件内部的零偏 */
    s_gyro_bias[0] = ((float)sum[0] / (float)samples) / MPU6050_GYRO_LSB_PER_DPS;
    s_gyro_bias[1] = ((float)sum[1] / (float)samples) / MPU6050_GYRO_LSB_PER_DPS;
    s_gyro_bias[2] = ((float)sum[2] / (float)samples) / MPU6050_GYRO_LSB_PER_DPS;
    return HAL_OK;
}
HAL_StatusTypeDef BspMpu_GetData(MpuData_t *data)
{
    MpuRaw_t raw = {0};
    if (BspMpu_ReadRaw(&raw) != HAL_OK)
    {
        return HAL_ERROR;
    }
    BspMpu_RawToData(&raw, data);
    /* 只减陀螺零偏：加速度和温度的"零"是物理零，不存在零偏概念，减了反而错 */
    data->gyro[0] -= s_gyro_bias[0];
    data->gyro[1] -= s_gyro_bias[1];
    data->gyro[2] -= s_gyro_bias[2];
    return HAL_OK;
}
