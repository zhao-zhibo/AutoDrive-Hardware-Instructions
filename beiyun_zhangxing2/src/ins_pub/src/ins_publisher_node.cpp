#include "RealNposSensor.h"
#include <cmath>
#include <unistd.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include<sensor_msgs/NavSatFix.h>
#include <INSPVAXA_Type.h>
#include "conversions.h"
#include "tf/tf.h"

using namespace navicore;

UnionStruct m_pUnionStruct;
ros::Publisher imu_raw_pub;
ros::Publisher ins_raw_pub;
ros::Publisher gps_raw_pub;
ros::Publisher ins_cov_pub;
std::ofstream fout("/home/fan/imu_raw.txt");
double roll,pitch,yaw;
RealNposSensor::RealNposSensor()
{

    //G320N  PwrPak7-E1   PwrPak7D-E1   SMART7-S
    //gro_scalefactor=(0.008/65536)/125*deg2rad;
    //acc_scalefactor=((0.200/65536)/125)*0.00980665;

    //NPOS220内置惯导比例因子
    // gro_scalefactor = (0.0151515 / 65536.0) / 200.0 * (M_PI / 180);
    // acc_scalefactor = ((0.400 / 65536) / 200.0) * 0.00980665;

    //bynav内置惯导比例因子
    gro_scalefactor = 3.0517578125e-05 * (M_PI / 180);
    acc_scalefactor = 3.74094009399414e-06;
    //NPOS320内置惯导比例因子
    //gro_scalefactor = deg2rad/65.5;
    //acc_scalefactor = 9.8080/8192.0; 
     bytes_remaining_ = 0;	//!< bytes remaining to be read in the current message
     buffer_index_ = 0;	//!< index into data_buffer_
     header_length_ = 0;	//!< length of the current header being read
     msg_length_ = 0;	        //!< length of the current header being read

    ////IMU-CPT  IMU-KVH1750
    //gro_scalefactor=0.1/(3600.0*256.0);
    //acc_scalefactor=0.05/32768;
    //m_SensorType = INS;

    memset(data_buffer_, 0, sizeof(data_buffer_)); //init buffer
}

RealNposSensor::~RealNposSensor()
{
}

bool RealNposSensor::open()
{

    // Open the Serial Port at the desired hardware port.
    //    //sudo chmod 777 /dev/ttyUSB0  read and write permission
    serial_port.Open("/dev/ttyUSB0", std::ios_base::in | std::ios_base::out); //open serial_port

    //    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(BaudRate::BAUD_921600); //set the baudrate of serial port

    //    // Set the number of data bits.
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    //    // Turn off hardware flow control.
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    //    // Disable parity.
    serial_port.SetParity(Parity::PARITY_NONE);

    //    // Set the number of stop bits.
    serial_port.SetStopBits(StopBits::STOP_BITS_1);

    return true;
}

bool RealNposSensor::isOpen()
{
    return serial_port.IsOpen();
}

void RealNposSensor::close()
{
    serial_port.Close();
}
//将从串口去读的二进制数据转化为IMU数据
void RealNposSensor::ParseBinary(unsigned char *message, size_t length, BINARY_LOG_TYPE message_id, sensor_msgs::Imu &imu_raw, ins_pub::INSPVAXA_Type &inspvaxa_info,sensor_msgs::NavSatFix &navsatfix,ins_pub::INSCOV &inscov)
{
    if (message_id == INSPVAXA_LOG_TYPE)
    { //INSPVAXA数据
        ByINSPVAXA frame;
        memset(&frame, 0, sizeof(ByINSPVAXA));
        memcpy(frame.header.sync_bytes, message, 3);
        unsigned short N = (frame.header.header_len = message[3]);
        frame.header.msg_ID = message[4] | (message[5] << 8);
        memcpy(&frame.header.week, message+14, 2);
        memcpy(&frame.header.ms, message + 16, 4);
        memcpy(&frame.slo_status, message + N, 4);
        memcpy(&frame.pos_type, message + N+4, 4);
        memcpy(&frame.latitude, message + N + 8, 8);
        memcpy(&frame.longitude, message + N + 16, 8);
        memcpy(&frame.altitude, message + N + 24, 8); // altitude means height
        memcpy(&frame.undulation,message+N+32,4);
        memcpy(&frame.veL_N, message + N + 36, 8);
        memcpy(&frame.vel_E, message + N + 44, 8);
        memcpy(&frame.vel_U, message + N + 52, 8);
        memcpy(&frame.roll, message + N + 60, 8);
        memcpy(&frame.pitch, message + N + 68, 8);
        memcpy(&frame.azimuth, message + N + 76, 8);
        memcpy(&frame.lat_std,message+N+84,4);
        memcpy(&frame.lon_std,message+N+88,4);
        memcpy(&frame.alt_std,message+N+92,4);
        memcpy(&frame.vN_std, message + N + 96, 4);
        memcpy(&frame.vE_std, message + N + 100, 4);
        memcpy(&frame.vU_std, message + N + 104, 4);
        memcpy(&frame.roll_std, message + N + 108, 4);
        memcpy(&frame.pitch_std, message + N + 112, 4);
        memcpy(&frame.azimuth_std, message + N + 116, 4);
        memcpy(&frame.ext_sol_stat, message + N + 120, 4);
        frame.checksum = message[N + 126] | (message[N + 127] << 8) |
                         (message[N + 128] << 16) | (message[N + 129] << 24);

        m_pUnionStruct.data_type = 1;
        m_pUnionStruct.week = frame.header.week;
        m_pUnionStruct.second = frame.header.ms*0.001;

        /****ROS Publishs the information of location*****/
       // ros::Time stamp(m_pUnionStruct.second-18+(m_pUnionStruct.week*7*24*3600)+315964800);
        inspvaxa_info.header.stamp.fromSec(m_pUnionStruct.second-18+(m_pUnionStruct.week*7*24*3600)+315964800); 
        inspvaxa_info.pos_type = frame.pos_type;
        inspvaxa_info.slo_status = frame.slo_status;
        inspvaxa_info.latitude = frame.latitude;
        inspvaxa_info.longitude = frame.longitude;
        inspvaxa_info.altitude = frame.altitude;
        inspvaxa_info.roll = frame.roll;
        inspvaxa_info.pitch = frame.pitch;
        inspvaxa_info.yaw = frame.azimuth;
        roll = frame.roll;
        pitch = frame.pitch;
        yaw = frame.azimuth;
        std::string zone;
        gps_common::LLtoUTM(inspvaxa_info.latitude, inspvaxa_info.longitude, inspvaxa_info.x, inspvaxa_info.y, zone);
        //ROS_INFO("UTM x is [%.12f], y is [%.12f], yaw is [%f]", inspvaxa_info.x, inspvaxa_info.y, inspvaxa_info.yaw);
        ins_raw_pub.publish(inspvaxa_info);

         ROS_INFO("COM:::::pos_type:::::  %d, slo_status::::::  %d",  inspvaxa_info.pos_type,inspvaxa_info.slo_status);
        navsatfix.header.stamp =  inspvaxa_info.header.stamp;//.fromSec(m_pUnionStruct.second-18+(m_pUnionStruct.week*7*24*3600)+315964800); 
        navsatfix.altitude = frame.altitude;
        navsatfix.longitude = frame.longitude;
        navsatfix.latitude = frame.latitude;
	    navsatfix.position_covariance[0] = frame.lat_std*frame.lat_std;
	    navsatfix.position_covariance[4] = frame.lon_std*frame.lon_std;
	    navsatfix.position_covariance[8] = frame.alt_std*frame.alt_std;
        if(frame.pos_type ==INS_RTKFIXED)
        {
            navsatfix.position_covariance_type = sensor_msgs::NavSatStatus::STATUS_FIX;
        }else
        {
            navsatfix.position_covariance_type = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }
        gps_raw_pub.publish(navsatfix);

        inscov.header.stamp= inspvaxa_info.header.stamp; 
        inscov.ros_time = ros::Time::now().toSec();
        inscov.latitude = frame.latitude;
        inscov.longitude = frame.longitude;
        inscov.altitude = frame.altitude;
        inscov.la_cov = frame.lat_std;
        inscov.lon_cov = frame.lon_std;
        inscov.high_cov = frame.alt_std;
        inscov.N_vel = frame.veL_N;
        inscov.E_vel = frame.vel_E;
        inscov.U_vel = frame.vel_U;
        inscov.E_vel_cov =frame.vE_std;
        inscov.N_vel_cov =frame.vN_std;
        inscov.U_vel_cov =frame.vU_std;
        inscov.roll = frame.roll;
        inscov.pitch = frame.pitch;
        inscov.yaw = frame.azimuth;
        inscov.roll_cov =frame.roll_std;
        inscov.pitch_cov =frame.pitch_std;
        inscov.yaw_cov =frame.azimuth_std;
        inscov.pos_type = frame.pos_type;
        inscov.slo_status = frame.slo_status;
        inscov.Undulation = frame.undulation;
        ins_cov_pub.publish(inscov);

        //std::cout << "[IMU] accx:"  << m_pUnionStruct.rpy_r << "[IMU] accy:" << m_pUnionStruct.rpy_p << "[IMU] accz:" <<   m_pUnionStruct.rpy_y << std::endl;
    }
    else if (message_id == RAWIMUA_LOG_TYPE)
    { //IMU数据
        RAWIMUA frame_imu;
        memset(&frame_imu, 0, sizeof(RAWIMUA));
        memcpy(frame_imu.header.sync_bytes, message, 3);
        unsigned short M = (frame_imu.header.header_len = message[3]);
        frame_imu.header.msg_ID = message[4] | (message[5] << 8);

        memcpy(&frame_imu.header.week, message+14, 2);
        memcpy(&frame_imu.header.ms, message + 16, 4);
        memcpy(&frame_imu.week, message + M, 4);
        memcpy(&frame_imu.seconds, message + M+4, 8);
        memcpy(&frame_imu.imu_status, message + M+12, 4);
        memcpy(&frame_imu.z_accel, message + M + 16, 4);
        memcpy(&frame_imu.y_accel, message + M + 20, 4);
        memcpy(&frame_imu.x_accel, message + M + 24, 4); // altitude means height
        memcpy(&frame_imu.z_gyro, message + M + 28, 4);
        memcpy(&frame_imu.y_gyro, message + M + 32, 4);
        memcpy(&frame_imu.x_gyro, message + M + 36, 4);
        ros::Time stamp(frame_imu.seconds-18+(frame_imu.week*7*24*3600)+315964800);
        imu_raw.header.stamp = stamp;
        imu_raw.header.frame_id = "imu-link";
        //四元数位姿,所有数据设为固定值，可以自己写代码获取IMU的数据，，然后进行传递
        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
        imu_raw.orientation.x = q.x;
        imu_raw.orientation.y = q.y;
        imu_raw.orientation.z = q.z;
        imu_raw.orientation.w = q.w;
        //线加速度
        imu_raw.linear_acceleration.x = (float)(frame_imu.x_accel * acc_scalefactor);
        imu_raw.linear_acceleration.y = (float)(-frame_imu.y_accel * acc_scalefactor);
        imu_raw.linear_acceleration.z = (float)(frame_imu.z_accel * acc_scalefactor);
        //角速度
        imu_raw.angular_velocity.x = (float)(frame_imu.x_gyro * gro_scalefactor);
        imu_raw.angular_velocity.y = (float)(-frame_imu.y_gyro * gro_scalefactor);
        imu_raw.angular_velocity.z = (float)(frame_imu.z_gyro * gro_scalefactor);
        imu_raw_pub.publish(imu_raw); 
        // ROS_INFO("IMUtimestamp is%lf, acc x is [%f],acc y is [%f],acc z is [%f]",imu_raw.header.stamp.toSec(), imu_raw.linear_acceleration.x, imu_raw.linear_acceleration.y, imu_raw.linear_acceleration.z);
    }
    else
    {
        std::cout << "No Data Input! Please input into command~" << std::endl;
    }
}

//从buffer中获取二进制数据并进行转化
void RealNposSensor::BufferIncomingData(unsigned char *message, sensor_msgs::Imu &imu_raw, ins_pub::INSPVAXA_Type &inspvaxa_info,sensor_msgs::NavSatFix &navsatfix,ins_pub::INSCOV &inscov, unsigned int length)
{
    // add incoming data to buffer
    for (unsigned int ii = 0; ii < length; ii++)
    {
        // make sure bufIndex is not larger than buffer
        if (buffer_index_ >= MAX_OUT_SIZE)
        {
            buffer_index_ = 0;
            //log_warning_("Overflowed receive buffer. Buffer cleared.");
        }

        if (buffer_index_ == 0)
        { // looking for beginning of message
            if (message[ii] == NOVATEL_SYNC_BYTE_1)
            { // beginning of msg found - add to buffer
                // ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXX");
                data_buffer_[buffer_index_++] = message[ii];
                bytes_remaining_ = 0;
            }
        }
        else if (buffer_index_ == 1)
        { // verify 2nd character of header
            if (message[ii] == NOVATEL_SYNC_BYTE_2)
            { // 2nd byte ok - add to buffer
                data_buffer_[buffer_index_++] = message[ii];
            }
            else
            {
                // start looking for new message again
                buffer_index_ = 0;
                bytes_remaining_ = 0;
            } 
        }
        else if (buffer_index_ == 2)
        { 
            if (message[ii] == NOVATEL_SYNC_BYTE_3)
            { // 2nd byte ok - add to buffer
                data_buffer_[buffer_index_++] = message[ii];
                msg_length_ = message[ii];
            }
             else
             {
                // start looking for new message again
                buffer_index_ = 0;
                bytes_remaining_ = 0;
            } 
        }
        else if (buffer_index_ == 3)
        {
            data_buffer_[buffer_index_++] = message[ii];
            header_length_ =message[ii];
            // msg_length_ = message[ii];
        }
        else if (buffer_index_ == 4)
        { 
                data_buffer_[buffer_index_++] = message[ii];
        }
        else if (buffer_index_ == 5)
        { // get message id
            data_buffer_[buffer_index_++] = message[ii];
            message_id_ = BINARY_LOG_TYPE(((data_buffer_[buffer_index_ - 1]) << 8) + data_buffer_[buffer_index_ - 2]);
            // bytes_remaining_ = msg_length_+6;
        }
        else if (buffer_index_ == 6)
        { 
                data_buffer_[buffer_index_++] = message[ii];
        }
        else if (buffer_index_ == 7)
        { 
                data_buffer_[buffer_index_++] = message[ii];
        }
                else if (buffer_index_ == 8)
        { 
                data_buffer_[buffer_index_++] = message[ii];
        }
                else if (buffer_index_ == 9)
        { 
                data_buffer_[buffer_index_++] = message[ii];
                msg_length_ =((data_buffer_[buffer_index_ - 1]) << 8) + data_buffer_[buffer_index_ - 2];
                bytes_remaining_ = header_length_+msg_length_-10;
        }
        else if (bytes_remaining_ == 1)
        { // add last byte and parse
            data_buffer_[buffer_index_++] = message[ii];
            // BINARY_LOG_TYPE message_id = (BINARY_LOG_TYPE) (((data_buffer_[5]) << 8) + data_buffer_[4]);
            // log_info_("Sending to ParseBinary")
            ParseBinary(data_buffer_, buffer_index_, message_id_, imu_raw, inspvaxa_info, navsatfix,inscov);
            // reset counters
            buffer_index_ = 0;
            bytes_remaining_ = 0;
        }
        else
        { // add data to buffer
            data_buffer_[buffer_index_++] = message[ii];
            bytes_remaining_--;
        }
        //std::cout<< "bytes_remaining_ length:" << bytes_remaining_<<std::endl;
    } // end for
}

int main(int argc, char *argv[])
{
    // init ros node
    ros::init(argc, argv, "imu_publisher_node"); // node
    ros::NodeHandle nh;
    imu_raw_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
    ins_raw_pub = nh.advertise<ins_pub::INSPVAXA_Type>("/gnss/ins_data", 10);
    gps_raw_pub = nh.advertise<sensor_msgs::NavSatFix>("/fix", 10);
    ins_cov_pub = nh.advertise<ins_pub::INSCOV>("/gnss/INSCOV",10);
    ros::Rate loop_rate(100);
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    //ready reading the data
    RealNposSensor pRealNposSensor;
    size_t ms_timeout = 50;
    if (!pRealNposSensor.open())
    {
        std::cout << "open serial port fail! " << std::endl;
        return 0;
    }
    else
    {
        std::cout << "open serial port success! " << std::endl;
    }

    // ensure the data is available
    while (!pRealNposSensor.serial_port.IsDataAvailable())
    {
        sleep(1);
    }

    sensor_msgs::Imu imu_raw;
    sensor_msgs::NavSatFix navsatfix;
    ins_pub::INSPVAXA_Type inspvaxa_info;
    ins_pub::INSCOV inscov;

    while (ros::ok())
    {
        //reading data
        DataBuffer buffer;
        try
        {
            pRealNposSensor.serial_port.Read(buffer, 100, 11); //串口读取数据
            //printf("\nwating buffer length is %d \n",buffer.size());
         }
         catch (ReadTimeout)
         {
                //printf("\nbuffer length is%d\n",buffer.size());
        }
         unsigned char message[MAX_OUT_SIZE]={0};
         memcpy(message,  &buffer[0], buffer.size());
         pRealNposSensor.BufferIncomingData(message, imu_raw, inspvaxa_info,navsatfix,inscov, buffer.size());
        ros::spinOnce();

    }
    return 0;
}
