#ifndef _MAHONY_AHRS_H_
#define _MAHONY_AHRS_H_ 


#include "string.h"
#include "math.h"
#include "stdint.h"
#define Mahony_RAD_TO_DEG 57.2957795f

//parameter setting
#define Mahony_Kp 1.0f
#define Mahony_Ki 0.0f

typedef struct{
    float q0,q1,q2,q3;
    float integralErr_Bx, integralErr_By, integralErr_Bz;
    float recipNorm;
    float invSampleFreq;
    float roll_Mahony, pitch_Mahony, yaw_Mahony;
    int8_t is_Init;
}   Mahony_t;

void MahonyAHRS_init(float ax,float ay,float az,float gx,float gy,float gz,Mahony_t *MahonyAHRS);
void MahonyAHRS_update(float gx,float gy,float gz,float ax,float ay,float az,Mahony_t *MahonyAHRS);
void Mahony_computeAngles(Mahony_t *M);

#endif