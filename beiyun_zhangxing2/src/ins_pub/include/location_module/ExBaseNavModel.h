#ifndef EXBASENAVMODEL_H
#define EXBASENAVMODEL_H

#include "navislam_global.h"
#include "BaseNavTime.h"
#include "ExQuadLikeliHoodLayer.h"
#include "ExGaussNewtonMatcher.h"

namespace navicore
{

class ExBaseNavModel : public BaseNavTime
{
public:
    ExBaseNavModel();
   ~ExBaseNavModel(void);
    void setExQuadLikeliHoodLayer(ExQuadLikeliHoodLayer* pQuadLikeliHoodLayer);
protected:
   virtual bool Init();
   virtual void Clear();
   virtual void onTimeUpdate(long epoch);
    bool scanMatch(XYZ_ScanLine& scanline);

    PoseState  m_PoseState;  //current pose
    PoseState  m_PoseState_paint;  //current pose
    ExQuadLikeliHoodLayer*  m_pExQuadLikeliHoodLayer;
    ExGaussNewtonMatcher*   m_pGaussNewtonMatcher;
};

}
#endif // EXBASENAVMODEL_H
