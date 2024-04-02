#ifndef NAVISLAM_H
#define NAVISLAM_H

#include <string.h>
#include <vector>
#include <stdio.h>
#include<iostream>
#include <list>
#include<math.h>

#if defined(NAVISLAM_LIBRARY)
#  define NAVISLAMSHARED_EXPORT __attribute__((visibility("default")))
#else
#  define NAVISLAMSHARED_EXPORT __attribute__((visibility("default")))
#endif


#define  PI        3.141592653589793
#define  deg2rad    PI/180.0f
#define  rad2deg    180.0f/PI
#define F_WGS84     (1./298.257223563)
#define R_WGS84         6378137.0E+00


//add by wjr
//constant value
const double a1 = 6378137.0;        /* earth semimajor axis (WGS84) (m) */
const double b1 = 6356752.3142;
const double f = 1.0 / 298.2572236;  /* earth flattening (WGS84) 地球椭球扁率*/
const double we = 7.292115147e-5;    //地球自转角速度
const double e2 = 1 - (b1 / a1)*(b1 / a1);     //
const double gm = 3986004.418e+8;    //地球引力常数
const double j2 = 1.082626683e-3;
const double j3 = -2.5327e-6;
const double e_2 = 1 - (b1 / a1)*(b1 / a1);
const double e_2_ = -1 + (a1 / b1)*(a1 / b1);
const double g0 = 9.7803267714;
//单位统一
typedef struct tagUnitconversion {
    double g0 = 9.7803267714;          // gravitational force
    double mg = 1.0e-3* g0;            // milli g
    double ug = 1.0e-6* g0;            // micro g
    double mGal = 1.0e-3*0.01;         // milli Gal = 1cm / s ^ 2 ~= 1.0E-6*g0
    double ugpg2 = ug / (g0*g0);       // ug / g ^ 2
    double ppm = 1.0e-6;               // parts per million
    double deg = PI / 180;             // arcdeg
    double min = deg / 60;             // arcmin
    double sec = min / 60;             // arcsec
    double hur = 3600;                 // time hour(1hur = 3600second)
    double dps = PI / 180 / 1;         // arcdeg / second
    double dph = deg / hur;            // arcdeg / hour
    double dpss = deg / sqrt(1);        // arcdeg / sqrt(second)
    double dpsh = deg / sqrt(hur);      // arcdeg / sqrt(hour)
    double dphpsh = dph / sqrt(hur);    // (arcdeg / hour) / sqrt(hour)
    double Hz = 1 / 1;                  // Hertz
    double dphpsHz = dph / Hz;          // (arcdeg / hour) / sqrt(Hz)
    double ugpsHz = ug / sqrt(Hz);      // ug / sqrt(Hz)
    double ugpsh = ug / sqrt(hur);      // ug / sqrt(hour)
    double mpsh = 1 / sqrt(hur);        // m / sqrt(hour)
    double mpspsh = 1 / 1 / sqrt(hur);  // (m / s) / sqrt(hour), 1 * mpspsh~= 1700 * ugpsHz
    double ppmpsh = ppm / sqrt(hur);    // ppm / sqrt(hour)
} Unitconversion;


#define POSITIVE_INFINITY  99999999999

#define SCAN_SIZE  10000

#define  NV_UINT64 uint64_t
#define  NV_UINT32 uint32_t
#define  NV_UINT16 uint16_t
#define  NV_UINT8  uint8_t


//ENVELOPE
typedef struct tagBound{
  double	m_lfXO;
  double	m_lfYO;
  double    m_lfZO;
  double	m_lfXE;
  double	m_lfYE;
  double    m_lfZE;

  double area2d()
  {
     return (m_lfXE - m_lfXO)*(m_lfYE - m_lfYO);
  }

  bool contains(double x,double y,double z=0){ return (x >= m_lfXO) &&
                                             (x <= m_lfXE) &&
                                             (y >= m_lfYO) &&
                                             (y <= m_lfYE) &&
                                             (z >= m_lfZO) &&
                                             (z <= m_lfZE)
                                             ;}
  void add(double x,double y,double z=0){

      if(m_lfXO != POSITIVE_INFINITY && m_lfYO!= POSITIVE_INFINITY)
      {
          if(x<m_lfXO)
              m_lfXO = x;

          if(y<m_lfYO)
              m_lfYO = y;

           if(z<m_lfZO)
              m_lfZO = z;

          if(x>m_lfXE)
              m_lfXE = x;

          if(y>m_lfYE)
              m_lfYE = y;

          if(z>m_lfZE)
             m_lfZE = z;
      }
      else
      {
           m_lfXO = x;
           m_lfYO = y;
           m_lfXE = x;
           m_lfYE = y;
           m_lfZO = z;
           m_lfZE = z;
      }
  }

  bool intersect(struct tagBound env)
  {
      //°üº¬ÇøÓò
        if(m_lfXO<=env.m_lfXO&&
           m_lfYO<=env.m_lfYO&&
           m_lfXE>= env.m_lfXE &&
           m_lfYE>= env.m_lfYE)
        {
           return true;
        }
        //µØÍŒÇøÓò°üº¬ÊÓ¿Ú
        else if(m_lfXO>=env.m_lfXO&&
                m_lfYO>=env.m_lfYO&&
                m_lfXE <= env.m_lfXE &&
                m_lfYE<= env.m_lfYE)
        {
            return true;
        }
        else
        {
            double b1x1, b1y1, b1x2, b1y2; // coords of the first box
            double b2x1, b2y1, b2x2, b2y2; // coords of the second box
        //    double nx1, ny1, nx2, ny2; // coords of the new box

            b1x1 = m_lfXO;
            b1y1 = m_lfYO; // bottom left
            b1x2 = m_lfXE;
            b1y2 = m_lfYE; // top right

            b2x1 =env.m_lfXO;
            b2y1 = env.m_lfYO; // bottom left
            b2x2 = env.m_lfXE;
            b2y2 =env.m_lfYE; // top right

            // do they meet
            if (b1x1 > b2x2 || b2x2 < b1x1
                || b1y1 > b2y2 || b2y2 < b1y1
                || b1x2 < b2x1 || b1y2 < b2y1) {
                return false;
            }

       /*     // find the left edge
            if (b1x1 < b2x1) {
                nx1 = b2x1;
            } else {
                nx1 = b1x1;
            }

            // find the right edge
            if (b1x2 > b2x2) {
                nx2 = b2x2;
            } else {
                nx2 = b1x2;
            }
            // find the top edge
            if (b1y2 > b2y2) {
                ny2 = b2y2;
            } else {
                ny2 = b1y2;
            }

            // find the bottom edge
            if (b1y1 > b2y1) {
                ny1 = b1y1;
            } else {
                ny1 = b2y1;
            }*/

           return true;

        }
     return false;
  }


} Bound;

//PoseState
typedef struct tagPoseState{
    double  m_dUtcTime;
    double	m_dPos[3];
    double	m_dVel[3];
    double	m_dCbn[9];
    double	m_dQbn[4];
    double  m_dRPY[3];  //ROTATION  roll(x) pitch(y) yaw(z)
    double  m_dRMSError[6];  //
    bool     m_bGNSSFixed; //FIX Result
} PoseState;

typedef struct tagPOINT_XYI
{
  float    m_dX;
  float    m_dY;
}POINT_XY;


typedef struct tagPOINT3D
{
  float    m_dX;
  float    m_dY;
  float    m_dZ;
}POINT_XYZ;

typedef struct tagLIDAR_FRAME
{
  std::vector<POINT_XYZ> m_points;
  double   m_time;
}LIDAR_FRAME;

typedef struct tagPOINT_LXYZI
{
  int              m_iLineID;      //scan line id
  unsigned short   m_iIntensity;
  float    m_dX;
  float    m_dY;
  float    m_dZ;
}POINT_LXYZI;

typedef struct tagPOINT_XYZI{
    unsigned short m_iIntensity;
    float    m_dX;
    float    m_dY;
    float    m_dZ;
}POINT_XYZI;

typedef std::vector<POINT_XY>  XY_ScanLine;
typedef std::vector<POINT_XYZ>  XYZ_ScanLine; //
typedef std::vector<POINT_XYZI>  ScanLine3D; //
typedef std::vector<POINT_LXYZI> LXYZI_ScanLine;//


#endif // NAVISLAM_H
