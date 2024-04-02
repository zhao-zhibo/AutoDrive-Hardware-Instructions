#ifndef REALNPOSSENSOR
#define REALNPOSSENSOR
#include "SerialPort.h"
#include <queue>
#include <mutex>
#include <thread>
#include "NovatelEnums.h"

#include <string.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include<sensor_msgs/NavSatFix.h>
#include "ins_pub/INSPVAXA_Type.h"
#include "ins_pub/INSCOV.h"
using namespace LibSerial;

namespace navicore {

class NovatelBinaryParse{
public:
    int BinaryParse(unsigned char* message,size_t len,std::queue<INSPVAXA> INSPVAXA_quuen,std::queue<RAWIMUSA> RAWIMUSA_quuen);
};

class RealNposSensor
{
public:
    RealNposSensor();
    ~RealNposSensor();
    bool open();
    bool isOpen();
    void close();
    SerialPort serial_port;
    //std::queue<INSPVAXA> INSPVAXA_quuen;
    //std::queue<RAWIMUSA> RAWIMUSA_quuen;
    std::queue<UnionStruct> UnionStruct_quuen;   

    void BufferIncomingData(unsigned char *message, sensor_msgs::Imu& imu_raw, ins_pub::INSPVAXA_Type& inspvaxa_info,sensor_msgs::NavSatFix &navsatfix,ins_pub::INSCOV &inscov, unsigned int length);
    void ParseBinary(unsigned char *message, size_t length, BINARY_LOG_TYPE message_id, sensor_msgs::Imu& imu_raw, ins_pub::INSPVAXA_Type& inspvaxa_info,sensor_msgs::NavSatFix &navsatfix,ins_pub::INSCOV &inscov);
    unsigned char data_buffer_[MAX_OUT_SIZE];	//!< data currently being buffered to read

    //unsigned char* data_read_;		//!< used only in BufferIncomingData - declared here for speed
    size_t bytes_remaining_;	//!< bytes remaining to be read in the current message
    size_t buffer_index_;	//!< index into data_buffer_
    size_t header_length_;	//!< length of the current header being read
    size_t msg_length_;	        //!< length of the current header being read

    BINARY_LOG_TYPE message_id_;	//!< message id of message currently being buffered

    void readGnss_IMU(UnionStruct& m_UnionStruct);         //read the data of IMU 
    void readGnss_IMU_all(std::vector<UnionStruct>& all_UnionStruct);    //read the vector of IMU 

    double gro_scalefactor;
    double acc_scalefactor;
};
}


#endif // REALNPOSSENSOR
