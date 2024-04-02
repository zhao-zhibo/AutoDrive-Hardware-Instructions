#pragma once
#include "navislam_global.h"
#include "ExQuadLikeliHoodLayer.h"
#include "transform.h"

namespace navicore
{

    class NAVISLAMSHARED_EXPORT ExGaussNewtonMatcher
	{
	 public:
		 ExGaussNewtonMatcher(ExQuadLikeliHoodLayer* pExQuadLikeliHoodLayer);
		~ExGaussNewtonMatcher();
        bool match(XYZ_ScanLine scanline, PoseState& poseState);
	private:
        bool subLayerMatch(int level, XYZ_ScanLine& scanline, double* qbn, double* pos , double* covr);
		void makediffS(double* xyz, double* diffS);
		ExQuadLikeliHoodLayer* m_pExQuadLikeliHoodLayer;
	};

}

