#include <stdio.h>
#include <ros/ros.h>
#include <iau_ros_msgs/VehicleFeed.h>
#include "iau_ros_msgs/Location.h"
#include "GaussProjection.h"
#include <LoopThread.h>
// #include <CANHandler.h>
#include "CANHandler.h"
#include <time.h>
#include <signal.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


using namespace ZCANBusSocket;

struct wheelInfo {
    ros::Time timestamp;
    double lfspd;
    double lbspd;
    double rfspd;
    double rbspd;
    int gear;
};

enum DAOYUAN {
    INS_Acc = 0x500,
    INS_GYRO = 0x501,
    INS_HeadingPitchRoll = 0x502,
    INS_HeightAndTime = 0x503,
    INS_LatitudeLongitude = 0x504,
    INS_Speed = 0x505,
    INS_DataInfo = 0x506,
    INS_Std = 0x507,
    INS_UTC = 0x508
};

enum EGear : unsigned int {
	UNKNOWN = 0,
	// P = 1,
	// R = 2,
	// N = 3,
	// D = 4,
	// I = 5     //invalid
	P = 1,
	R = 2,
	N = 3,
	D = 4,
	I = 5
};

ZBaseLoopThread *thWrite;
ZCANBusSocket::CANHandler *canHandler;
wheelInfo wheelFeed;
ros::Time last_time;

float d_pitch = 0, d_roll = 0, d_yaw = 0;
float d_ve = 0, d_vn = 0, d_vz = 0;
int d_type = 0;
double d_secs = 0;
double d_height = 0;
int d_week = 0;
double d_lat = 0, d_lon = 0;
int8_t g_iYear = 0, g_iMonth = 0, g_iDay = 0, g_iHour = 0, g_iMin = 0, g_iSec = 0;
int16_t g_iMsec = 0;
int32_t g_iNsec = 0;
std::shared_ptr<std::ofstream> pWrite_stream;
std::mutex pWrite_mutex;
bool b_endFrame = false;
ros::Publisher* m_locpub;


void CANMessageCallback(const ZCANBusSocket::CANMessage *msg, ZCANBusSocket::CANStatus status) {
    // std::cout << "callback" <<std::endl;
    if (status == 0) {
        // //把接收消息保存 txt
        // std::lock_guard<std::mutex> lock(pWrite_mutex);
        // char info[256];
        // sprintf(info, "%f\t0x%x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\n", 
        // ros::Time::now().toSec(), msg->id,
        // msg->msg[0], msg->msg[1], msg->msg[2], msg->msg[3],
        // msg->msg[4], msg->msg[5], msg->msg[6], msg->msg[7]);
        // *pWrite_stream << info;

        switch (msg->id) {
            case INS_HeadingPitchRoll:
                d_pitch = (((u_int16_t)(msg->msg[0]) << 8) + msg->msg[1]) * 0.010986 - 360;
                d_roll  = (((u_int16_t)(msg->msg[2]) << 8) + msg->msg[3]) * 0.010986 - 360;
                d_yaw   = (((u_int16_t)(msg->msg[4]) << 8) + msg->msg[5]) * 0.010986 - 360;
                if (d_yaw < 0) {
                    d_yaw += 360.0;
                }
                break;
            case INS_Speed:
                d_vn  = (((u_int16_t)(msg->msg[0]) << 8) + msg->msg[1]) * 0.0030517 - 100;
                d_ve  = (((u_int16_t)(msg->msg[2]) << 8) + msg->msg[3]) * 0.0030517 - 100;
                d_vz  = (((u_int16_t)(msg->msg[4]) << 8) + msg->msg[5]) * 0.0030517 - 100;
                break;
            case INS_DataInfo:
                d_type = msg->msg[0];
                // std::cout << d_type << std::endl;
                break;
            case INS_HeightAndTime:
                d_height = (((u_int32_t)(msg->msg[0]) << 24) + ((u_int32_t)(msg->msg[1]) << 16) + ((u_int32_t)(msg->msg[2]) << 8)+ msg->msg[3]) * 0.001 - 10000;
                d_secs   = (((u_int32_t)(msg->msg[4]) << 24) + ((u_int32_t)(msg->msg[5]) << 16) + ((u_int32_t)(msg->msg[6]) << 8)+ msg->msg[7]);
                break;
            case INS_UTC:
                // d_week   = (((u_int32_t)(msg->msg[4]) << 24) | ((u_int32_t)(msg->msg[5]) << 16) | ((u_int32_t)(msg->msg[6]) << 8) | msg->msg[7]);
                g_iYear  = msg->msg[0];
                g_iMonth = msg->msg[1];
                g_iDay   = msg->msg[2];
                g_iHour  = msg->msg[3];
                g_iMin   = msg->msg[4];
                g_iSec   = msg->msg[5];
                g_iMsec  = (msg->msg[6] << 8) + msg->msg[7];
                g_iNsec = g_iMsec * (10^6);
                break;
            case INS_LatitudeLongitude:
                d_lat    = (((u_int32_t)(msg->msg[0]) << 24) | ((u_int32_t)(msg->msg[1]) << 16) | ((u_int32_t)(msg->msg[2]) << 8) | msg->msg[3]) * 1E-7 - 180;
                d_lon    = (((u_int32_t)(msg->msg[4]) << 24) | ((u_int32_t)(msg->msg[5]) << 16) | ((u_int32_t)(msg->msg[6]) << 8) | msg->msg[7]) * 1E-7 - 180;

                b_endFrame = true; 
                break;
            default:
                break;
        }

        // ROS_INFO("%02x", msg->id);
    }

}

void publish_pos_dy(){
        // 高斯投影
        LBtoxy coord(d_lon, d_lat);
        coord.calculateAll();
        double gaussX = coord.getx();
        double gaussY = coord.gety() + 500000.0;
        
        // 发布 Location
        iau_ros_msgs::Location loc;
        // 使用 GPS 时间
        //double seconds = d_secs;
        //gtime_t t = gst2time((uint32_t)g_iSec, g_iNsec);
        
        loc.timestamp = ros::Time((uint32_t)g_iSec, g_iNsec);
        loc.wgs84_pos[0] = d_lon;
        loc.wgs84_pos[1] = d_lat;
        loc.gau_pos[0] = gaussX;
        loc.gau_pos[1] = gaussY;
        loc.wgs84_pos[2] = d_height;
        loc.speed[0] = d_ve;  // bestpos.east_velocity;
        loc.speed[1] = d_vn; // bestpos.north_velocity;
        loc.speed[2] = d_vz; // bestpos.up_velocity;
        loc.orientation[0] = d_pitch; // bestpos.pitch;
        loc.orientation[1] = d_roll; // bestpos.roll;
        loc.orientation[2] = d_yaw;

        if (d_type == 16) {
            loc.INS_status = "single";
        } else if (d_type == 50){
            loc.INS_status = "RTK_Fixed";
        } else if (d_type == 32){
            loc.INS_status = "RTK_Float";
        } else if (d_type == 17){
            loc.INS_status = "DGPS";
        }
        //printf("location: %.3lf %.7lf %.7lf %.1lf %s\n", g_iSec, loc.wgs84_pos[0], loc.wgs84_pos[1],loc.orientation[2], loc.INS_status.c_str());
        std::cout << "time:     " << g_iSec << "lon:    " << loc.wgs84_pos[0] << "lat:    " << loc.wgs84_pos[1]  << "yaw:     " <<loc.orientation[2] << "status:      "  << loc.INS_status<<std::endl;
        if (m_locpub && loc.wgs84_pos[0] != 0.0) {
            loc.header.stamp = ros::Time::now();
            m_locpub->publish(loc);
        }
}

bool WriteThread() {
    // if(wheelFeed.timestamp == last_time) {
    //     ROS_INFO("timestamp has not changed.");
    //     return false;
    // }
    //  wheelFeed.lbspd = 1;
    // wheelFeed.lfspd = 2;
    // wheelFeed.rbspd = 3;
    // wheelFeed.rfspd = 3;
    // wheelFeed.gear = 1;
    // wheelFeed.timestamp = msg->timestamp;

    CANMessage message[3];
    message[0].type = 0;
    message[0].length = 8;
    memset(message[0].msg, 0, sizeof(unsigned char) * 8);
    message[0].id = 0x348;

    message[1].type = 0;
    message[1].length = 8;
    memset(message[1].msg, 0, sizeof(unsigned char) * 8);
    message[1].id = 0x34A;

    message[2].type = 0;
    message[2].length = 8;
    memset(message[2].msg, 0, sizeof(unsigned char) * 8);
    message[2].id = 0x1F5;

    uint16_t lf = uint16_t(wheelFeed.lfspd / 0.03125);
    message[0].msg[0] |= (lf >> 8);
    message[0].msg[1] |= (lf & 0xFF);

    uint16_t rf = uint16_t(wheelFeed.rfspd / 0.03125);
    message[0].msg[2] |= (rf >> 8);
    message[0].msg[3] |= (rf & 0xFF);

    uint16_t lb = uint16_t(wheelFeed.lbspd / 0.03125);
    message[1].msg[0] |= (lb >> 8);
    message[1].msg[1] |= (lb & 0xFF);

    uint16_t rb = uint16_t(wheelFeed.rbspd / 0.03125);
    message[1].msg[2] |= (rb >> 8);
    message[1].msg[3] |= (rb & 0xFF);

    uint8_t gear = uint8_t(wheelFeed.gear);
    message[2].msg[3] |= gear;

    CANStatus &&st = canHandler->Write(message, 3);
    last_time = wheelFeed.timestamp;

     //把发送消息保存 txt
    // std::lock_guard<std::mutex> lock(pWrite_mutex);
    // for (int i = 0; i < 3; i++){
    //     char info[256];
    //     // if (i == 0) {
    //     //     std::cout << "id = " << std::setw(2) << std::hex << std::setfill('0') << message[i].id << "\t" << (uint16_t)(uint8_t)message[i].msg[0] 
    //     //     << "\t" << (uint16_t)(uint8_t)message[i].msg[3] << "\t" << (uint16_t)(uint8_t)message[i].msg[5] << "\t" << (uint16_t)(uint8_t)message[i].msg[6] << std::endl;
    //     // }
    //     // std::cout << "id = " << std::hex << message[i].id << endl;
    //     sprintf(info, "%f\t0x%x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\t%02x\n", 
    //     ros::Time::now().toSec(), message[i].id, 
    //     message[i].msg[0], message[i].msg[1], message[i].msg[2], message[i].msg[3],
    //     message[i].msg[4], message[i].msg[5], message[i].msg[6], message[i].msg[7]);
    //     *pWrite_stream << info;
    // }

    return true;
}

void vehiclefeedCallback(const iau_ros_msgs::VehicleFeed::ConstPtr& msg) {
    wheelFeed.lbspd = msg->LBspd;
    wheelFeed.lfspd = msg->LFspd;
    wheelFeed.rbspd = msg->RBspd;
    wheelFeed.rfspd = msg->RFspd;
    wheelFeed.gear = msg->gear;
    wheelFeed.timestamp = msg->timestamp;
    // ROS_INFO("lb = %f,lf = %f, rb = %f, rf = %f", wheelFeed.lbspd, wheelFeed.lfspd, wheelFeed.rbspd, wheelFeed.rfspd);
}

int attach() {
    canHandler = new ZCANBusSocket::CANHandler(CANType::SOCKET_CAN);
    if (0 != canHandler->OpenChannel(0, CANRate::CAN_RATE_500K, 0)) {
        delete canHandler;
        canHandler = nullptr;
        return -1;
    }
    canHandler->ReadLoop(std::bind(CANMessageCallback,
                   std::placeholders::_1, std::placeholders::_2),
    10);
    thWrite = new ZBaseLoopThread(std::bind(WriteThread));
    thWrite->SetLoopInterval(20);
    thWrite->Start();
    return 1;
}

void signalHandler(int sig) {
    ROS_INFO("shutdown.");
    ros::shutdown();
    exit(0);
}



int main(int argc, char** argv) {
    
    ros::init(argc, argv, "pos_dy_node");
    ros::NodeHandle n;
    ros::Subscriber sub_vehicle = n.subscribe("/IAU/WheelSpeed", 10, vehiclefeedCallback);
    ros::Publisher  pub    = n.advertise<iau_ros_msgs::Location>("IAU/Location", 10);
    m_locpub = &pub;
    std::cout << "abcd" << std::endl;
    pWrite_stream = std::make_shared<std::ofstream>(
        "ins_"+std::to_string(ros::Time::now().toSec())+".txt", std::ios::out|std::ios::binary);
    
    ROS_INFO("check.");
    
    signal(SIGINT, signalHandler);
    attach();

    while(1) {
        ros::spinOnce();
        if(b_endFrame){
        publish_pos_dy();
        b_endFrame = false;
        }
    }

    return 0;
}
