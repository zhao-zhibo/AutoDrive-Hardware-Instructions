#pragma once
#include "navislam_global.h"
#include "Envelope.h"
#include "ProbabilityValue.h"

#define QUADINX  int64_t

namespace navicore
{

#define PATCH_CELL_SIZE  1000
#define NULL_VALUE      -9999

/*
  四叉树节点
*/
class  NAVISLAMSHARED_EXPORT QuadNode
{
public:
	QuadNode(void);
	~QuadNode(void);

	bool contains(double x,double y){ return (x >= m_envNode.m_lfXO) && 
	                                         (x <= m_envNode.m_lfXE) &&
										     (y >= m_envNode.m_lfYO) &&
										     (y <= m_envNode.m_lfYE);}

	bool intersect(double x,double y,double w,double h,Envelope& intersect);
	void meanfilter(int n);
	void thresholdfilter(unsigned char threshold);
	void setLLHValue(double x,double y);
    void setLLHValue(double x,double y,float value);
    void setInnerLLHValue(double x,double y);
	float getValue(double x, double y);
	void clearContent();

	Bound     m_envNode;
	QUADINX	  m_iIndex;
	int       m_iLevel;
	double    m_dx;
    double    m_dy;
	double m_dCenterX;
	double m_dCenterY;
    unsigned char*  m_bValueArray;  //地块数组
//子节点
	QuadNode* m_pChild[4];  //00 01 10 11
	QuadNode* m_pParent;    //父节点

private:
	inline unsigned char probility2Index(unsigned short p);
	inline unsigned short index2probility(unsigned char idx);
    inline float odds(float p){ return p/(10000.0f-p); }
};

}
