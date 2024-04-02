#pragma once
#include"navislam_global.h"
#include "navislam_global.h"
#include <string>

namespace navicore{
// ´«¸ÐÆ÷
class  Sensor
{
public:
    enum SENSOR_TYPE {LIDAR,INS,GNSS};
	Sensor(void);
	virtual ~Sensor(void);
    virtual bool open();	virtual bool isOpen();
	virtual void close();
	SENSOR_TYPE sensorType(){ return m_SensorType;}
    void setName(std::string name);
    std::string name(){ return m_sName;}
protected:
	SENSOR_TYPE m_SensorType;
    std::string  m_sName;  //Ãû³Æ
};

}

