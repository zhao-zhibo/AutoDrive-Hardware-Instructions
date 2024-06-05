#pragma once
#include <stdlib.h>
#include <math.h>
#include "navislam_global.h"
#include<vector>


#define S(a) a*a 
#define F_WGS84     (1./298.257223563)
#define R_WGS84         6378137.0E+00


namespace  navicore{



class  matlib
{
public:
	//��˹�ֲ������
	static double gaussrand(double E,double V);
	static double normPDF(double z); //��׼��̬�ֲ�����
	static double normDense(double z);//��׼��̬�ֲ������ܶ�

	//��������
	static void  matrixZero(double* a, int m, int n); //Zero Matrix
	static void  matrixEye(double* a, int n);    // Indentity Matrix
	static void	 matrixDiag(double* v, int n, double* a); //Diagonal Matrix
	static void	 matrixTranspose(double* a, double* b, int m, int n);	//Transpose matrix
	static int	 matrixInverse(double *av,int n,double*a); //inverse matrix
	static void	 matrixAntiSymmetic(double* v, double *m); //3x3 dimension Anti-symmetric Matrix
	static int   matrixCholesky(double* a, int n);	//Cholesky Decomposition(Note: here get up triangular part )
	static void	 matrixMutiply(double* a, double* b, double* c, int row, int m, int col);	//Matrix Mutiply
	static void  matrixDiagMutiply(double* a, double* vecDiag, double* c, int row,int col);  //Matrix Mutiply Diag Vector
	static void  matrixReal(double *a, double real, double *b, int row, int col);	//Real Mutiply Matrix
	static void	 matrixAdd(double* a, double* b, double* c, int row, int col); //Matrix Addition
    static void	 matrixSub(double* a, double* b, double* c, int row, int col); //Matrix Subtraction
	static double matrixNorm(double* a, int row, int col); //Matrix norm
	static double matrixDet(double* a, int n);  //determinant ����ʽ
	//��������
	static double vectorDot(double* a, double* b, int n);	//Vector dot Mutiply
	static void   vectorCross(double* a, double* b, double* c); //Vector Cross Product
	static void   vectorRotation(double* rpyAngle,double* a, double* b); //vector rotation in 3d
	static void   vectorNormal(double* v,double* nv, int n); //vector normalize	
	static void Vector2Quat(double* a, double* b);
	static double  vectorNorm(double* v, int n);
	static void quat2pos(double* q, double* pos);
	static void quatInverse(double* q, double* q_inv);
	static void Quat2Vector(double *q, double *RotationVector);
	static void Pos2Quat(double lat, double lon, double* q_en);
	
	//3D��תת��
	static void euler2matrix(double* euler, double* matrix);
	static void matrix2euler(double* matrix,double* euler);
	static void matrix2quat(double* matrix, double* q);
	static void euler2quat(double* euler,double* q);
	static void quat2matrix(double* q, double* matrix);
	static void quat2euler(double* q, double* euler);
	static void quatrotvec(double* q, double* n,double* rn);
	static void quatprod(double* q, double* p, double *qp);
	static void quattrans(double* q, double* qt); 
	static void quatslerp(double*q, double* p, double t, double* qt); //Interpolates between two quaternions using spherical linear interpolation  t>0 && t<1
	//�Ƕ�Normalize
	static inline double normalize_angle_pos(double angle)
	{
		return fmod( fmod(angle,2.0f* PI) + 2.0F*PI,2.0f*PI);
	}
	static inline double normalize_angle(double angle)
	{
		double a = normalize_angle_pos(angle);
		if(a >PI){
			a -=2.0f*PI;
		}
		return a;
	}
	static inline double normalize_angleDegree(double angle)
	{
		return normalize_angle(angle/180*PI)/PI*180;
	}

	static double radAngleNorm(double angle);
	
	//��ֵ�Ƚ�
	static int  iMin(int a, int b){
	   return (((a) < (b)) ? (a) : (b));
	};
	static int  iMax(int a, int b){
	   return (((a) > (b)) ? (a) : (b));
	};
	static double  dMin(double a, double b){
	   return (((a) < (b)) ? (a) : (b));
	};
	static double  dMax(double a, double b){
	   return (((a) > (b)) ? (a) : (b));
	};

	//���uvd��xyz����ת��
	static void uvd2xyz(double* fxy,double* cxy, double* uvd, double* xyz);
	static void xyz2uvd(double* fxy,double* cxy, double* xyz, double* uvd);
	static unsigned char rgb2gray(unsigned char r,unsigned char g,unsigned char b);

	static double calvariance(std::vector<double> a);
	static double calmean(std::vector<double> a);
	

	template <typename T>
	T static clamp(const T value, const T min, const T max) {
	  if (value > max) {
		return max;
	  }
	  if (value < min) {
		return min;
	  }
	  return value;
	};


	static void ecef2blh(double *pos, double *blh);
	static void blh2ecef(double *blh, double *pos);
	static void blh2enu(double *rblh, double *bblh, double *enu);
	static void enu2ecef(double *rblh, double *enu, double* xyz);
	static void xyz2ned(double *pos, double *E);
	static void ned2xyz(double *pos, double *ned, double *pointECEF);
	static void pos2ned(double *rpos, double *bpos, double *ned);
};
}
