#include "MahonyAHRS.h"
//-------------------------------------------------------------------------------
//MahonyAHRS算法为一种用于姿态解算的显式互补滤波算法
//我一直不懂这个单词的意思，今天学了之后才懂了(
//BMI088为六轴传感器无磁力计，通过测量获取的加速度和角速度进行解算
//-------------------------------------------------------------------------------

static float invSqrt(float x)
{
    volatile float y;
    memcpy(&y, &x, sizeof(x));
    y = 1.0f / sqrtf(y);
    return y;
}

//快速倒数平方根(为无FPU准备，可优化计算)
//在带FPU运算单元时可以直接用1.0f/sqrtf(x)
float Mahony_invsqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i;
    memcpy(&i, &y, sizeof(y));
    i = 0x5f1ffff9 - (i>>1);
    memcpy(&y, &i, sizeof(i));
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
}

//互补滤波初始化函数，获得初始四元数
void MahonyAHRS_init(float ax,float ay,float az,float gx,float gy,float gz,Mahony_t *MahonyAHRS)
{
    float recipNorm;
    float init_yaw,init_pitch,init_roll;
    float cr2,cp2,cy2,sr2,sp2,sy2;
    float sin_roll,cos_roll,sin_pitch,cos_pitch;
    float magX,magY;
    float q0,q1,q2,q3;

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    //磁力计传入数据处理(我们BMI088没有磁力计)
    if((gx != 0.0f) && (gy != 0.0f) && (gz != 0.0f))
    {
        recipNorm = invSqrt(gx * gx + gy * gy + gz * gz);
        gx *= recipNorm;
        gy *= recipNorm;
        gz *= recipNorm;
    }
    init_pitch = atan2f(-ax,az);
    init_roll = atan2f(ay,az);

    sin_roll = sinf(init_roll);
    cos_roll = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    if((gx != 0.0f) && (gy != 0.0f) && (gz != 0.0f))
    {
        magX = gx * cos_pitch + gy * sin_roll * sin_pitch + gz * cos_roll * sin_pitch;
        magY = gy * cos_roll - gz * sin_roll;
        init_yaw = atan2f(-magY,magX);
    }else{
        init_yaw = 0.0f;
    }
    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    //归一化四元数
    recipNorm = invSqrt(q0 *q0 + q1 * q1 + q2 * q2 + q3 * q3);
    MahonyAHRS->q0 = q0 * recipNorm;
    MahonyAHRS->q1 = q1 * recipNorm;
    MahonyAHRS->q2 = q2 * recipNorm;
    MahonyAHRS->q3 = q3 * recipNorm;
    MahonyAHRS->recipNorm = recipNorm;
}

//六轴数据更新
void MahonyAHRS_update(float gx,float gy,float gz,float ax,float ay,float az,Mahony_t *MahonyAHRS)
{
    float recipNorm;
    float halfvx,halfvy,halfvz;
    float halfex,halfey,halfez;
    float qa,qb,qc;

    if(!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = MahonyAHRS->q1 * MahonyAHRS->q3 - MahonyAHRS->q0 * MahonyAHRS->q2;
        halfvy = MahonyAHRS->q0 * MahonyAHRS->q1 + MahonyAHRS->q2 * MahonyAHRS->q3;
        halfvz = MahonyAHRS->q0 * MahonyAHRS->q0 - 0.5f + MahonyAHRS->q3 * MahonyAHRS->q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        //启用积分项
        if(Mahony_Ki >= 0.0f)
        {
            MahonyAHRS->integralErr_Bx += Mahony_Ki * halfex * MahonyAHRS->invSampleFreq;
            MahonyAHRS->integralErr_By += Mahony_Ki * halfey * MahonyAHRS->invSampleFreq;
            MahonyAHRS->integralErr_Bz += Mahony_Ki * halfez * MahonyAHRS->invSampleFreq;
            gx += MahonyAHRS->integralErr_Bx;
            gy += MahonyAHRS->integralErr_By;
            gz += MahonyAHRS->integralErr_Bz;
        }else{
            MahonyAHRS->integralErr_Bx = 0.0f;
            MahonyAHRS->integralErr_By = 0.0f;
            MahonyAHRS->integralErr_Bz = 0.0f;
        }

        gx += Mahony_Kp * halfex;
        gy += Mahony_Kp * halfey;      
    }
    //欧拉公式求解初值
    gx *= (0.5f * MahonyAHRS->invSampleFreq);
    gy *= (0.5f * MahonyAHRS->invSampleFreq);
    gz *= (0.5f * MahonyAHRS->invSampleFreq);
    qa = MahonyAHRS->q0;
    qb = MahonyAHRS->q1;
    qc = MahonyAHRS->q2;
    MahonyAHRS->q0 += (-qb * gx - qc * gy - MahonyAHRS->q3 * gz);
    MahonyAHRS->q1 += (qa * gx + qc * gz - MahonyAHRS->q3 * gy);
    MahonyAHRS->q2 += (qa * gy - qb * gz + MahonyAHRS->q3 * gx);
    MahonyAHRS->q3 += (qa * gz + qb * gy - qc * gx);
    //求范数，公式为1/sqrt(q0^2+q1^2+q2^2+q3^2)
    recipNorm = invSqrt(MahonyAHRS->q0 * MahonyAHRS->q0 + MahonyAHRS->q1 * MahonyAHRS->q1 + MahonyAHRS->q2 * MahonyAHRS->q2 + MahonyAHRS->q3 * MahonyAHRS->q3);
    MahonyAHRS->q0 *= recipNorm;
    MahonyAHRS->q1 *= recipNorm;
    MahonyAHRS->q2 *= recipNorm;
    MahonyAHRS->q3 *= recipNorm;
}

void Mahony_computeAngles(Mahony_t *M)
{
    M->roll_Mahony = atan2f(M->q0 * M->q1 + M->q2 * M->q3, 0.5f - M->q1 * M->q1 - M->q2 * M->q2) * Mahony_RAD_TO_DEG;
    M->pitch_Mahony = asinf(-2.0f * (M->q1 * M->q3 - M->q0 * M->q2)) * Mahony_RAD_TO_DEG;
    M->yaw_Mahony = atan2f(M->q1 * M->q2 + M->q0 * M->q3, 0.5f - M->q2 * M->q2 - M->q3 * M->q3) * Mahony_RAD_TO_DEG;
}