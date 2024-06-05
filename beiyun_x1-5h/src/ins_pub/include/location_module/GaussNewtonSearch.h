#pragma once
#include "navislam_global.h"
#include "QuadLikeliHoodLayer.h"

/*
    高斯牛顿迭代法搜索
*/

namespace navicore {

class GaussNewtonSearch
{
public:
	GaussNewtonSearch(QuadLikeliHoodLayer* pLikeliHoodMap);
	~GaussNewtonSearch(void);
    bool search(float* pScanLineX,float* pScanLineY, int scanlen,double& X, double& Y, double& Phi);
    bool search(XY_ScanLine& scanline, PoseState& position);
protected:
    bool subLayerSearch(int level, XY_ScanLine& scanline, double& X, double& Y, double& Phi,double* cov);
    void makediffS(double  diffS[][3],double x,double y,double phi);
	QuadLikeliHoodLayer* m_pLikeliHoodMap;
    double m_dlastYaw;
};


}
