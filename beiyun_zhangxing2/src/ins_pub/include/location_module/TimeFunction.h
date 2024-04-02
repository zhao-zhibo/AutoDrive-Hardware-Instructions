#ifndef TIMEFUNCTION
#define TIMEFUNCTION
#include <time.h>
#include <math.h>
#include <iostream>
#include <string.h>

#define MAXLEAPS 64
static double leaps[MAXLEAPS+1][7]={
    {2017,1,1,0,0,0,-18},
    {2015,7,1,0,0,0,-17},
    {2012,7,1,0,0,0,-16},
    {2009,1,1,0,0,0,-15},
    {2006,1,1,0,0,0,-14},
    {1999,1,1,0,0,0,-13},
    {1997,7,1,0,0,0,-12},
    {1996,1,1,0,0,0,-11},
    {1994,7,1,0,0,0,-10},
    {1993,7,1,0,0,0,-9},
    {1992,7,1,0,0,0,-8},
    {1991,1,1,0,0,0,-7},
    {1990,1,1,0,0,0,-6},
    {1988,1,1,0,0,0,-5},
    {1985,7,1,0,0,0,-4},
    {1983,7,1,0,0,0,-3},
    {1982,7,1,0,0,0,-2},
    {1981,7,1,0,0,0,-1},
    {0},
};
static double gpst0[]={1980,1,6,0,0,0};

typedef struct{
    time_t time;
    double sec;
}gtime_t;

typedef struct{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    double second;
}UTC_TIME;

typedef struct{
    int week_num;
    double second_in_week;
}GPSTIME;

int read_leaps_txt(FILE *fp);
int read_leaps(const char* file);

gtime_t epoch2time(double* ep);
gtime_t gpst2time(int week,double second);
gtime_t timeadd(gtime_t t,double second);
double timediff(gtime_t t1,gtime_t t2);

gtime_t GPSTime2UTCTime(int week,double sec,double leapsec);
gtime_t utc2gpst(gtime_t t);

double time2gpst(gtime_t t,int *week);


#endif // TIMEFUNCTION

