#ifndef LIVOXREALLIDARSENSOR_H
#define LIVOXREALLIDARSENSOR_H
#include "Sensor.h"
#include <queue>
#include <mutex>
#include "TimeFunction.h"
#include <string.h>
#include <iostream>
#include <fstream>

//#include "livox_sdk.h"
#include "../../sdk_core/include/livox_sdk.h"
namespace navicore{

#define BROADCAST_CODE_LIST_SIZE  1

typedef enum{
    kDeviceStateDisconnect = 0,
    kDeviceStateConnect = 1,
    kDeviceStateSampling = 2
} DeviceState;

typedef struct{
    uint8_t handle;
    DeviceState device_state;
    DeviceInfo  info;
} DeviceItem;


typedef struct taglvxScanline
{
    XYZ_ScanLine* pScanline;
    double timestamp;
}lvxScanline;

class LivoxRealLiDARSensor : public Sensor
{
public:
    enum LIDARTYPE{FRONT_LIDAR=0x01,REAR_LIDAR=0x02};
    LivoxRealLiDARSensor();
     ~LivoxRealLiDARSensor();
    virtual bool open();
    virtual bool isOpen();
    virtual void close();

    XYZ_ScanLine* readScanLine(LIDARTYPE liType, double& timeStamp);


};


}

#endif // LIVOXREALLIDARSENSOR_H
