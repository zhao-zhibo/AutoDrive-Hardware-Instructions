#pragma once
#include "navislam_global.h"


namespace navicore
{

//model = 0,1,2,3 respectly presented BJ54,XIAN80,WGS84,CGCS2000 coordiate
class GaussProjCal
{
public:
	static int XYZ_BLH(int model, double XYZcoord[3], double BLH[3]);  //XYZ to BLH 
	static int BLH_XYZ(int model, double BLH[3], double XYZcoord[3]);  //BLH to XYZ
	static double DU_DMS(double coord);
	static double DMS_DU(double coord);
	static double Rad_DU(double coord);
	static double DU_Rad(double coord);
	static void RAD_DMS(double Rad, int &Deg, int &Min, double &Sec);
    static void ElilpExpend(double& a,double f,double H,double BO,double& dB);

	static void GaussInvProj(double a, double f, double longitude0,
                             double X, double Y, double &longitude, double &latitude);
	static void  GaussProjInvCal(int coordsys, double longitude0,
                                 double X, double Y, double H, double &longitude, double &latitude);

	static void GaussProjCalxyz(int m_ncoord, double longitude0,
		double longitude, double latitude, double H, double &X, double &Y);
};


}
