#pragma once

#ifndef COMMON_BASENAVTIME_H
#define COMMON_BASENAVTIME_H

#include "navislam_global.h"
#include <thread>
#include <vector>
#include <mutex>
#include "NavVisitor.h"

namespace navicore {

class NAVISLAMSHARED_EXPORT BaseNavTime
{
public:
	BaseNavTime(void);
	~BaseNavTime(void);
	 void addEpochVisitor(IEpochVisit*  pEpochVisit);
	 void removeEpochVisitor(IEpochVisit*  pEpochVisit);
	 void onStart();
	 void onPause();
	 bool isPaused();
	 void onStop();
     long currentEpoch(){return m_iCurrentEpoch;}
     void setCurrentEpoch(long epoch){m_iCurrentEpoch =  epoch;}
	 bool isActive();
protected:
	virtual void onTimeUpdate(long epoch);
	virtual bool Init();
	virtual void Clear();
	static  void run(BaseNavTime* pBaseNavTime);
     std::thread*  m_pthread;
     long          m_iCurrentEpoch;
     bool          m_bPause;
	 bool          m_bStop;
	 std::vector<IEpochVisit*>  m_vecEpochVisitor;
};

}  // namespace core

#endif 
