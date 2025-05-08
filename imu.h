
#pragma once

#include <math.h>
#include "pico/stdlib.h"
/*
    为研究温度对陀螺仪的零飘的影响，记录不同温度下校准的数据。
    温度    gyro0   gyro1   gyro2       日期
    27.99   -0.633  0.261   0.05342     20230628
    28.03   -0.630  0.275   0.06054     20230628
    31.09   -0.458  0.192   -0.05404    20230629
    35.54   -0.503  0.082   -0.10247    20230629
    35.08   -0.476  0.077   -0.08295    20230629 //和上一个测试之间保持静止，只是温度不同
    33.97   -0.418  0.071   -0.09123    20230629 //和上一个测试之间保持静止，只是温度不同
    33.42   -0.389  0.066   -0.09184    20230629 //和上一个测试之间保持静止，只是温度不同
*/
class imu_info
{
    public:
    uint32_t tmark; //系统时间戳，代表收到信息的时间
    uint64_t us_signal; //中断信号时间戳，代表着数据的最初时间

    float acce[3]; //传感器坐标加速度
    float gyro[3]; //角速度
    float angle[3]; //角度
    float acc_gnd[3]; //大地坐标下的加速度。

    float zaxis; //z轴和大地z轴夹角的cos值。可用来校正对地测距。
    float temprature; //温度


    imu_info() {

        tmark=0;
        us_signal=0;
        temprature=0;
        zaxis=1.0;
        for(size_t i=0;i<3;i++) {
            acce[i]=0;
            gyro[i]=0;
            angle[i]=0;
            acc_gnd[i]=0;
        }

    }


    imu_info& 
    operator+=(const imu_info &o)
    {
 
        for (size_t i = 0; i < 3; i++) {
			acce[i] += o.acce[i];
			gyro[i] += o.gyro[i];
			angle[i] += o.angle[i];
            acc_gnd[i]+= o.acc_gnd[i];

		}

        zaxis+= o.zaxis;

        return *this;
    }
 
    imu_info
    operator+(const imu_info &rhs)
    {
        imu_info sum;
        sum = *this;
        sum += rhs;
        return sum;
    }


    imu_info
    operator/(const int n)
    {
        imu_info sum;
        sum= *this;
        for(size_t i=0;i<3;i++) {
            sum.acce[i]/=n;
            sum.gyro[i]/=n;
            sum.angle[i]/=n;
            sum.acc_gnd[i]/=n;
        }

        sum.zaxis/=n;
        
        return sum;
    }

    imu_info& operator=(const imu_info& o)
    {
        tmark=o.tmark;
        us_signal=o.us_signal;
        temprature=o.temprature;
        zaxis=o.zaxis;

        for(size_t i=0;i<3;i++) {
            acce[i]=o.acce[i];
            gyro[i]=o.gyro[i];
            angle[i]=o.angle[i];
            acc_gnd[i]=o.acc_gnd[i];
        }
        return *this;
    } 
};


class Yaw_info
{
    public:
    Yaw_info() {
        us_signal=0;
        yaw=0;
        gyro[0]=0;
        gyro[1]=0;
        gyro[2]=0;
        temprature=0;
    }

    Yaw_info& operator=(const Yaw_info& o)
    {
        us_signal=o.us_signal;
        yaw=o.yaw;
        temprature=o.temprature;
        for(size_t i=0;i<3;i++) {
            gyro[i]=o.gyro[i];
        }
        return *this;
    } 

    uint64_t us_signal; //中断信号时间戳，代表着数据的最初时间
    float yaw;
    float temprature; //区间温度
    float gyro[3]; //区间的gyro。
};

class ImuBase
{
public:
    ImuBase() {
        //exInt=0;
       // eyInt=0;
       // ezInt=0;
        qnum[0]=1.0; qnum[1]=0.0; qnum[2]=0.0; qnum[3]=0.0;
    }

    void ResetYaw() {
        ///exInt=0;
        ///eyInt=0;
        ///ezInt=0;
        //将航向角设置为0;
        float angles[3];
        ToAngles(qnum, angles);
        angles[2]=0;
        ToQnum(angles, qnum);
    }
    
    //姿态角的定义：滚转角是Z轴与通过Y轴的铅锤面的夹角，俯仰角则是Y轴与水平面的夹角，Y轴投影到水平面与导航系北向的夹角即为航向角。
    //如果最先旋转Z轴，此时航向角等于欧拉角；旋转之后，绕X轴旋转，此时俯仰角和欧拉角相等；旋转之后，绕Y轴旋转，此时横摇角和欧拉角相等。
    //所以只有Z-X-Y的旋转顺序（注：导航系需为东-北-天定义），欧拉角和姿态角才是相同的；否则，欧拉角和姿态角就不相同了。
    //参与计算的加速度单位g 陀螺仪单位是弧度/s
    //计算时间在124微秒左右，invSqrt反而更慢
    //XY平面对水平面的夹角 = acos(cos(pitch)*cos(roll)); 如果pitch=45, roll=45, 算出来是60.
    //原始函数对accs做了归一处理，使得垂向加速度不能准确计算，需要修改，不允许修改原值。
    //https://github.com/jremington/MPU-9250-AHRS/blob/master/MPU9250_Mahony/MPU9250_Mahony.ino
    //这个来源里不返回错误。
    //当前的主要问题是，水平校准后，如果机身倾斜，则航向角漂移大增。水平旋转一圈，航向角不归位，有较大偏差。
    void calculate_euler_angles(float gyros[3], float accs[3], float angles[3], float acc_gnd[3], float& zaxis, float half_time)
    {
        //加速度计对陀螺仪的修正系数
        //减小这个数值时，水平倾角收敛缓慢。似乎是一个角度变化敏感性参数，小一点似乎yaw漂移小。
        //20240818 手持飞控摇晃测试，0.5比1.0好。
        #define Kp (1.0)  //原始1.0 加速度对角速度的补偿参数，不知道意义，严重影响三个倾角的计算。如果修改到0.01时，倾角严重偏差。
        //#define Ki (0.01)
        
        float dynamic_Kp=1.0;

        float vx, vy, vz;
        float ex, ey, ez;

        //20250327 尝试修正关于水平方向加速度的影响, 使用动态的kp值。
        float acc_norm= sqrt(accs[0]*accs[0]+accs[1]*accs[1]+accs[2]*accs[2]); //加速度的单位是G
        // if(acc_norm>1.414) dynamic_Kp=0.1;  //水平加速度>1G
        // else if(acc_norm>1.345) dynamic_Kp = 0.2; //9m/s2水平加速度。
        // else if(acc_norm>1.28) dynamic_Kp = 0.3; //8m/s2水平加速度。
        // else if(acc_norm>1.22) dynamic_Kp = 0.4; //7m/s2水平加速度。
        // else if(acc_norm>1.166) dynamic_Kp = 0.5; //6m/s2水平加速度。
        // else if(acc_norm>1.12) dynamic_Kp = 0.6; //5m/s2水平加速度。
        // else if(acc_norm>1.077) dynamic_Kp = 0.7; //4m/s2水平加速度。
        // else if(acc_norm>1.044) dynamic_Kp = 0.8; //3m/s2水平加速度。
        // else if(acc_norm>1.02) dynamic_Kp = 0.9; //2m/s2水平加速度。
        dynamic_Kp= 1.0/acc_norm;
        if(dynamic_Kp < 0.2) dynamic_Kp=0.2;
        if(dynamic_Kp > 1.0) dynamic_Kp=1.0;

        //https://github.com/jremington/MPU-9250-AHRS/blob/master/MPU9250_Mahony/MPU9250_Mahony.ino
        //这个来源并没有检查加速度返回失败。
        //if( accs[0]*accs[1]*accs[2]==0)
        //    return false;

        // /* 对加速度数据进行归一化处理 */
        // recipNorm= sqrt(accs_copy[0]* accs_copy[0] +accs_copy[1]*accs_copy[1] + accs_copy[2]*accs_copy[2]);
        // accs_copy[0] /= recipNorm;
        // accs_copy[1] /= recipNorm;
        // accs_copy[2] /= recipNorm;

        /* DCM矩阵旋转 */
        vx = 2*(qnum[1]*qnum[3] - qnum[0]*qnum[2]);
        vy = 2*(qnum[0]*qnum[1] + qnum[2]*qnum[3]);
        vz = qnum[0]*qnum[0] - qnum[1]*qnum[1] - qnum[2]*qnum[2] + qnum[3]*qnum[3];

        // /* 在机体坐标系下做向量叉积得到补偿数据 */
        //这些代码可能是校正gyro0,gyro1的漂移对yaw角的影响而做的。
        //gyro0,gyro1如果有数值，一定反应在加速度上。利用加速度的方向去矫正前者的漂移。
        
        ex = accs[1]*vz - accs[2]*vy;
        ey = accs[2]*vx - accs[0]*vz;
        ez = accs[0]*vy - accs[1]*vx;

        // /* 对误差进行PI计算，补偿角速度 */
        //https://github.com/jremington/MPU-9250-AHRS/blob/master/MPU9250_Mahony/MPU9250_Mahony.ino
        //这个来源计算的方法不同。
        // if(Ki>0.0)
        // {
        //     exInt += ex ;
        //     eyInt += ey ;
        //     ezInt += ez ;
        //     gyros[0] += Ki*exInt;
        //     gyros[1] += Ki*eyInt;
        //     gyros[2] += Ki*ezInt;
        // }


//当ki==0， kp=1.0时 简化为：
//简化后计算时间由124微秒减少为112微秒
//去除所有的陀螺仪调节似乎更好更稳定，这个只是在水平平面内测试。
//这么做的目的是抑制陀螺仪x,y轴上的漂移，因为真实的旋转必然导致重力加速度变化。而虚假的漂移量不会。

        // gyros[0] += Kp*ex;
        // gyros[1] += Kp*ey;
        // gyros[2] += Kp*ez;

        gyros[0] += dynamic_Kp*ex;
        gyros[1] += dynamic_Kp*ey;
        gyros[2] += dynamic_Kp*ez;

        //float half_time= float(ms)/2000.0f; //刷新时间的一半，单位秒

        /* 按照四元数微分公式进行四元数更新 */
        qnum[0] += (-qnum[1]*gyros[0] - qnum[2]*gyros[1] - qnum[3]*gyros[2])*half_time;
        qnum[1] += (qnum[0]*gyros[0] + qnum[2]*gyros[2] - qnum[3]*gyros[1])*half_time;
        qnum[2] += (qnum[0]*gyros[1] - qnum[1]*gyros[2] + qnum[3]*gyros[0])*half_time;
        qnum[3] += (qnum[0]*gyros[2] + qnum[1]*gyros[1] - qnum[2]*gyros[0])*half_time;

        float recipNorm = sqrt(qnum[0]*qnum[0] + qnum[1]*qnum[1] + qnum[2]*qnum[2] + qnum[3]*qnum[3]);
        for(size_t i=0;i<4;i++)   qnum[i]/=recipNorm; 


        //计算在大地坐标下三个方向的加速度，不同于传感器坐标。
            // float ang[3]; //姿态角，弧度，便于计算。
            // ToAngles(qnum, ang);
            // float mid = cos(ang[0])*cos(ang[1]);
            // //float za = acos(mid); //传感器z轴对铅垂线的夹角。
            // //传感器的加速度方向和坐标系相反，z轴向上，加速度向下为正。三个数据都是有向的，向下为正。
            // float acc_z_v = accs[2]*mid; //加速度z轴对地垂直分量。姿态平稳时，这个是主要数据。
            // float acc_x_v = -accs[0]*sin(ang[1]); //x轴加速度对地垂直分量，ang[1]是roll, 影响的是x轴的加速度
            // float acc_y_v = accs[1]*sin(ang[0]); //y轴加速度对地垂直分量，ang[0]是pitch, 影响的是y轴的加速度
            // acc_gv = acc_z_v+ acc_x_v + acc_y_v;

            //以上转角度再计算的算法可以通过四元数直接计算。
            float ay= 2*(qnum[0]*qnum[1] + qnum[2]*qnum[3]);
            float ax = 1- 2*(qnum[1]*qnum[1] + qnum[2]*qnum[2]);
            float saxay= sqrt(ax*ax+ay*ay);
            float cosa0= ax/saxay;  //绕x轴旋转的角度，pitch角度，即y轴对水平线的夹角的cos
            float sina0= ay/saxay;  //绕x轴旋转的角度，pitch角度，即y轴对水平线的夹角的sin

            float sina1= 2*(qnum[0]*qnum[2]- qnum[1]*qnum[3]); //绕y轴旋转的角度，roll角度，即x轴对水平线的夹角的sin
            float cosa1= sqrt(1-sina1*sina1); //绕y轴旋转的角度，roll角度，即x轴对水平线的夹角的cos
            float cosz = cosa0*cosa1; //传感器z轴对大地坐标z轴形成的夹角的余弦。

            zaxis=cosz;
            acc_gnd[2] = accs[2]*cosz - accs[0]*sina1 + accs[1]*sina0 - 1.0; //扣除重力干扰


            float sinz = sqrt(1-cosz*cosz); //传感器z轴对大地坐标z轴形成的夹角的正弦。

            float acc_z_h = (accs[2]-cosz)*sinz; //传感器加速度对大地坐标的水平分量。扣除重力的干扰
            float acc_x_h = (accs[0]+sina1)*cosa1; //x轴加速度对大地坐标的水平分量。扣除重力的干扰
            float acc_y_h = (accs[1]-sina0)*cosa0; //y轴加速度对大地坐标的水平分量。扣除重力的干扰


            //z轴加速度投影到水平面后，要继续分为x轴和y轴两个方向。这需要计算两个夹角。
            //如果pitch角度为0，投影完全落在x轴上，如果roll角度为0，投影完全落在y轴上。
            //如果roll>0, pitch>0, 投影落在xy平面的第三象限。
            //0, 0.2->0,  0,-0.2->PI ,  0.2, 0->PI/2,  -0.2, 0 -> -PI/2
            //0.2, 0.2->PI/4, 0.2, -0.2 -> 3PI/4, -0.2, 0.2-> -PI/4, -0.2, -0.2-> -3PI/4;
            float acc_z_h_s = atan2(sina0, sina1); //z轴的水平投影在水平面形成的角度
            float z_h_x= sin(acc_z_h_s); //x轴因子。
            float z_h_y= cos(acc_z_h_s); //y轴因子。
            float acc_z_h_x = acc_z_h * z_h_x;
            float acc_z_h_y = acc_z_h * z_h_y;

            acc_gnd[0] = acc_x_h + acc_z_h_x;
            acc_gnd[1] = acc_y_h + acc_z_h_y;


   
        ToAngles(qnum, angles);

        return;

        #undef Kp
        #undef Ki
    }


    //角度转四元数
    void ToQnum(float angles[3], float q[4])
    {
        //弧度。
        float pitch = angles[0] / 57.2958;
        float roll = angles[1] / 57.2958;
        float yaw = angles[2] / 57.2958;

        float cp= cosf(pitch*0.5);
        float cr= cosf(roll*0.5);
        float cy= cosf(yaw*0.5);
        float sp= sinf(pitch*0.5);
        float sr= sinf(roll*0.5);
        float sy= sinf(yaw*0.5);

        q[0] = cp*cr*cy + sp*sr*sy;
        q[1] = sp*cr*cy - cp*sr*sy;
        q[2] = cp*sr*cy + sp*cr*sy;
        q[3] = cp*cr*sy - sp*sr*cy;
    }

    //四元数转角度
    void ToAngles(float q[4], float angles[3])
    {
        //atan2(y, x)= arctan(y/x)
        angles[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]) , (1 - 2 * (q[1] * q[1] + q[2] * q[2])));
        angles[1] = asin(2*(q[0]*q[2] - q[1]*q[3]));
        angles[2] = atan2(2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]*q[2] + q[3] * q[3]));
        angles[0] *= 57.2958;
        angles[1] *= 57.2958;
        angles[2] *= 57.2958;
    }

protected:
    float qnum[4];
    //float exInt, eyInt, ezInt;
};