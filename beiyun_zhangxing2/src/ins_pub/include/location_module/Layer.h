#pragma once
#include "navislam_global.h"
#include <string>

namespace navicore {

class Layer
{
public:
    enum LAYER_TYPE{UNKNOWN_LAYER,LLHMAP_LAYER,VECTOR_LAYER};
	Layer(void);
	virtual~Layer(void);
	virtual bool update(long t);
    void setName(std::string name){m_sName = name;}
    std::string name(){ return m_sName;}
    virtual Bound bound(){return m_envBound;}
	LAYER_TYPE type();   // 图层类型
protected:
    std::string  m_sName; //名称
	LAYER_TYPE  m_LayerType;
	Bound  m_envBound;  //地图区域
};

}

