/*
* MPU6050 offset calibration. shimojo
* 拙いプログラムのため、ただの参考にしてください。間違いが多くあります。
*/

//Header files
#include <stdio.h>
#include "MPU6050_Offset_Calibration.h"
#include "TJ_MPU6050.h"
#define Cal_num  10 //オフセットを校正するための計測回数


//============================================================
// Functions
//------------------------------------------------------------
// Offset Caliburation

void MPU6050_Offset_Calibration(int *Ax_bias, int *Ay_bias, int *Az_bias, int *Gx_bias, int *Gy_bias, int *Gz_bias )
{
	  RawData_Def myAccelRaw, myGyroRaw; // add
	  //ScaledData_Def myAccelScaled, myGyroScaled; // add

    int Ax,Ay,Az;//Raw data
    int Gx,Gy,Gz;//Raw data
    int Ax_av, Ay_av, Az_av=0;
    int Gx_av, Gy_av, Gz_av=0;
    int i=0;

    Ax_av=0; Ay_av=0; Az_av=0;
    Gx_av=0; Gy_av=0; Gz_av=0;

// Get initial measure data
    while (i< Cal_num )
    {
        MPU6050_Get_Accel_RawData(&myAccelRaw);//この命令で加速度とジャイロを取得
        MPU6050_Get_Gyro_RawData(&myGyroRaw);//上の命令で取得したデータを渡すだけ

        Ax=myAccelRaw.x-*Ax_bias;  Ay=myAccelRaw.y-*Ay_bias; Az=myAccelRaw.z-*Az_bias;
	    Gx=myGyroRaw.x-*Gx_bias;  Gy=myGyroRaw.y-*Gy_bias; Gz=myGyroRaw.z-*Gz_bias;
	   Ax_av+=Ax; Ay_av+=Ay; Az_av+=Az;
	   Gx_av+=Gx; Gy_av+=Gy; Gz_av+=Gz;
	   i+=1;
       HAL_Delay(10); //delay 10ms
    }
    Ax_av=(float)Ax_av/Cal_num; Ay_av=(float)Ay_av/Cal_num;Az_av=(float)Az_av/Cal_num;
    Gx_av=(float)Gx_av/Cal_num; Gy_av=(float)Gy_av/Cal_num;Gz_av=(float)Gz_av/Cal_num;

    // calculate average of offset value & Correct for GyroData only.
   // *Ax_bias+=(float)Ax_av; *Ay_bias+=(float)Ay_av;*Az_bias+=(float)Az_av;
    *Gx_bias+=(float)Gx_av; *Gy_bias+=(float)Gy_av;*Gz_bias+=(float)Gz_av;
}


