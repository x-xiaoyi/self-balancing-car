#ifndef __ALGORITHEM_ATTITUDE_H__
#define __ALGORITHEM_ATTITUDE_H__
#include "bsp/bsp_mpu.h"
typedef struct
{
    float roll_reg;
    float pitch_reg;
    float yaw_reg;
}Attitude_t;
void Attitude_Init ( const MpuData_t *first);
void Attitude_Update ( const MpuData_t *imu, float dt_s, Attitude_t *out) ;
#endif