#pragma once
#include <stdint.h>
#define MPU6050_GYRO_FS_500 1
#define MPU6050_ACCEL_FS_4 1
#define MPU6050_DLPF_BW_42 3
extern double g_roll_deg, g_pitchchip_deg; extern double g_gyro_dps[3]; extern double g_bias[3]; extern double g_noise;
struct MPU6050 { void initialize(){} bool testConnection(){ return true; }
  void setFullScaleGyroRange(int){} void setFullScaleAccelRange(int){} void setDLPFMode(int){}
  void getMotion6(int16_t*ax,int16_t*ay,int16_t*az,int16_t*gx,int16_t*gy,int16_t*gz){
    double r=g_roll_deg/57.2958, p=g_pitchchip_deg/57.2958;
    // chip frame gravity: roll about X then pitch about Y
    *ax=(int16_t)(-sin(p)*8192); *ay=(int16_t)(sin(r)*cos(p)*8192); *az=(int16_t)(cos(r)*cos(p)*8192);
    double n = g_noise*((rand()%200)-100)/100.0;
    *gx=(int16_t)((g_gyro_dps[0]+n)*65.5+g_bias[0]); *gy=(int16_t)((g_gyro_dps[1]+n)*65.5+g_bias[1]); *gz=(int16_t)((g_gyro_dps[2]+n)*65.5+g_bias[2]); } };
