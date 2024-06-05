#pragma once
#include "navislam_global.h"

#define MIN_PROBABILITY_VALUE   1000
#define MAX_PROBABILITY_VALUE   9000
#define INTERVAL_PROBABILITY    32   //ги0.9-0.1)/254
#define PROBABILITY_SCALE       10000
#define HIT_PROBABILITY_VALUE   5500
#define MISS_PROBABILITY_VALUE  4500
namespace  navicore {

class ProbabilityValue
{
public:
	static  unsigned char probility2Index(float p);
	static  unsigned char probility2Index(unsigned short p);
	static  unsigned short index2probility(unsigned char idx);
	static  float index2probility2(unsigned char idx);
    static  float odds(float p){ return p/(10000.0f-p); }
};

}
