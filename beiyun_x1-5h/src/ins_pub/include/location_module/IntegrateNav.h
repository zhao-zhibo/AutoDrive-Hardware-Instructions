#ifndef INTEGRATENAV
#define INTEGRATENAV
#include "matlib.h"


namespace navicore {

typedef struct tagPosition{
    double m_dX;
    double m_dY;
    double m_dZ;
    double m_dPitch; //ROTATION Y
    double m_dRoll;  //ROTATION X;
    double m_dYaw;   //ROTATION Z;
    double m_dRMSError[2];
    double m_dR[9];
    double m_dT[3];
}Position;

typedef struct tagNAVIGATIONELEMENT
{
    double	pos[3];
    double	vel[3];
    double	cbn[9];
    double	qbn[4];   //
    double	qne[4];
    double	dv[3];
}NAVIGATIONELEMENT;

typedef struct tagPARAMETER
{
    double	Rm;    //卯酉圈曲率半径
    double	Rn;    //子午圈曲率半径
    double	g[3];
    double	wb[3];
    double	fb[3];
    double	wie[3];
    double	wen[3];
    double	fn[3];

}PARAMETER;


typedef struct tagOBSERVATION_IMU{
    double time;
    double gyro[3];
    double acce[3];
}OBSERVATION_IMU;

typedef struct tagNavState
{
    NAVIGATIONELEMENT m_navEle;
    PARAMETER         m_navPar;
    OBSERVATION_IMU m_imuobs;
    double LODO[3];
    double Lgps[3];
    double Llidar[3];
    double t;          //历元时刻
    double dt;         //t(k+1) - t(k)
    double dg[3];      //陀螺仪漂移随机分量
    double da[3];      //加速度计零偏随机分量
    double Tau;        //相关时间
} NavState;//导航状态量结构体

//
class INS
{
public:
   static void mech(OBSERVATION_IMU prev, OBSERVATION_IMU cur, NAVIGATIONELEMENT *nav, PARAMETER *par);
   static double Cal_N(double lat);
   static double Cal_M(double lat);
   static void Get_wie_n(double lat, double* wien);
   static void Get_wen_n(double lat, double h, double M, double N, double* Vel_NED, double* wenn);
   static double Cal_normalGravity(double lat, double h);
};


class EKFLooseCouple{
public:
    EKFLooseCouple(void);
    ~EKFLooseCouple(void);
    void setObsSTD2(double* AVRerror, double* std2Obs);
    //INS状态滤波预测
    void predictNavState(NavState s);
    //卡尔曼松组合滤波
    void geo2ecef(double e, double n, double *geo, double *r_e);
    NavState updateNavState(NavState s, double* obs,double* std, int type);
    //误差状态量清零
    void clearState();
    //imu采样间隔
    double imu_ts;
private:
    //误差状态量预测
    bool predict(double* x, double* P, double* F, double* Q);
    //观测向量更新
    bool update(double* x, double* P, double* z, double*H, double* R);
    //反馈更新
    void feedback(NavState *s, double* x);

    void makeF(NavState s, double F[][15]); //生成F矩阵
    void makePosZ(NavState s, double* obs, double* z);  //生成观测向量
    void makePosZ_lidar(NavState s, double* obs, double* z);
    void makePosH(NavState s, double H[3][15]);  //生成观测方程转移矩阵
    void makePosH_lidar(NavState s, double H[3][15]);
    void makeVelZ(NavState s, double* obs, double* z);  //生成速度观测向量
    void makeVelH(NavState s, double H[3][15]);  //生成速度观测方程转移矩阵
    void makeODOZ(NavState s, double* obs, double* z);  //生成观测向量
    void makeODOH(NavState s, double H[3][15]);  //生成观测方程转移矩阵
    void makeQ(NavState s, double FI[][15], double Q[][15]); //生成系统状态误差噪声矩阵
    void makeR(double* obs, double R[][3]);  //观测向量噪声矩阵

    double P[15][15];    //方差转移矩阵
    double x[15];        //误差状态向量 (r,v,w,g,a)
    double std2ga[12];  //姿态与速度测量噪声和陀螺仪加速度零漂方差

    //std::ofstream*  m_pLogStream;
};

}
#endif // INTEGRATENAV

