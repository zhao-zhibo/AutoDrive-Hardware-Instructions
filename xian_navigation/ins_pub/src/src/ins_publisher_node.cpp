#include "RealNposSensor.h"
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include<sensor_msgs/NavSatFix.h>
#include "ins_pub/INSPVAXA_Type.h"
#include "ins_pub/INSCOV.h"
#include "conversions.h"
#include "def_struct.h"
using namespace navicore;

UnionStruct m_pUnionStruct;
    ros::Publisher imu_raw_pub ;
    ros::Publisher ins_raw_pub;
    ros::Publisher gps_raw_pub ;
    ros::Publisher ins_cov_pub;
std::ofstream fout("/home/fan/imu_raw.txt");

double yaw = 0.0;
double vx = 0;
double vy = 0;
double vz = 0;


RealNposSensor::RealNposSensor()
{

    //G320N  PwrPak7-E1   PwrPak7D-E1   SMART7-S
    //gro_scalefactor=(0.008/65536)/125*deg2rad;
    //acc_scalefactor=((0.200/65536)/125)*0.00980665;

    //NPOS220内置惯导比例因子
    // gro_scalefactor = (0.0151515 / 65536.0) / 200.0 * (M_PI / 180);
    // acc_scalefactor = ((0.400 / 65536) / 200.0) * 0.00980665;

    //bynav内置惯导比例因子
    gro_scalefactor = 3.35276126861572e-05 * (M_PI / 180);
    acc_scalefactor = 4.65661287307739e-06;
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
    serial_port.SetBaudRate(BaudRate::BAUD_460800); //set the baudrate of serial port

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
void RealNposSensor::ParseBinary(unsigned char *message, size_t length, BINARY_LOG_TYPE message_id, sensor_msgs::Imu &imu_raw, ins_pub::INSPVAXA_Type &inspvaxa_info,sensor_msgs::NavSatFix &navsatfix,ins_pub::INSCOV &inscov) {

    int i = 0;
    char x;
    bool start_flag = 0;
    int n = 0;
    Gyro g;
    Acc a;
    Att at;
	Vn v;
	pos p;
    car_v car;
    Gps gps;
    int posType;
    int headType;
    int flag;
    char start[4];
    char temp[8];
    char jwg[16];
    double t_time= 0;
    // 增加message长度的判断
    int nLen = strlen( (char *)message );
    if (nLen < 239) {
        return;
    }
    
    while (i <= length) {
        x = message[i];
        if (start_flag == 0) {
            if (x == ' ') {
                continue;
            }

            switch (n) {
                case 0:
                    start[0] = x, n++;
                    break;
                case 1:
                    start[1] = x, n++;
                    break;
                case 2:
                    start[2] = x, n++;
                    break;
                case 3:
                    start[3] = x, n++;
                    break;
                default:
                    break;
            }
            if (start[0] == 'A' && start[1] == 'A' && start[2] == 'C' && start[3] == 'C') {  //AA CC
                start_flag = 1;

            } else if (n == 4) {
                start[0] = start[1];
                start[1] = start[2];
                start[2] = start[3];
                n = 3;
            }
        } else if (start_flag == 1) {
            if (n >= 4 && n <= 9) {              //Gyro_x
                temp[n - 4] = x;
                if (n == 9) {
                    g.G_x = str2int(temp, 6, 1) * 0.001;
                }
                n++;
            } else if (n >= 10 && n <= 15) {      //Gyro_y;
                temp[n - 10] = x;
                if (n == 15) {
                    g.G_y = str2int(temp, 6, 1) * 0.001;
                }
                n++;
            } else if (n >= 16 && n <= 21) {     //Gyro_z;
                temp[n - 16] = x;
                if (n == 21) {
                    g.G_z = str2int(temp, 6, 1) * 0.001;
                }
                n++;
            } else if (n >= 22 && n <= 27) {       //ACC_x
                temp[n - 22] = x;
                if (n == 27) {
                    a.A_x = str2int(temp, 6, 1) * 0.00001;
                }
                n++;
            } else if (n >= 28 && n <= 33) {      //Acc_y
                temp[n - 28] = x;
                if (n == 33) {
                    a.A_y = str2int(temp, 6, 1) * 0.00001;
                }
                n++;
            } else if (n >= 34 && n <= 39) {      //Acc_z
                temp[n - 34] = x;
                if (n == 39) {
                    a.A_z = str2int(temp, 6, 1) * 0.00001;
                }
                n++;
            } else if (n >= 40 && n <= 47) {
                temp[n - 40] = x;
                if (n == 47) {
                    t_time = str2int(temp, 8, 0) * 0.1;
                    for (int i = 0; i < 8; i++) {
                        temp[i] = ' ';
                    }
                }
                n++;
            } 
            else if (n >= 48 && n <= 51) {     //Att_x
				temp[n - 48] = x;
				if (n == 51) {
					at.Att_x = str2int(temp, 4, 1)*0.01;
				}
				n++;
			}
			else if (n >= 52 && n <= 55) {      //Att_y
				temp[n - 52] = x;
				if (n == 55) {
					at.Att_y = str2int(temp, 4, 1)*0.01;
				}
				n++;
			}
			else if (n >= 56 && n <= 59) {     //Att_z
				temp[n - 56] = x;
				if (n == 59) {
					at.Att_z = str2int(temp, 4, 1)*0.01;
				}
				n++;
			}

			else if (n >= 60 && n <= 65) {   //Vn_x
				temp[n - 60] = x;
				if (n == 65) {
					v.V_x = str2int(temp, 6, 1)*0.0001;
				}
				n++;
			}
			else if (n >= 66 && n <= 71) {    //Vn_y
				temp[n - 66] = x;
				if (n == 71) {
					v.V_y = str2int(temp, 6, 1)*0.0001;
				}
				n++;
			}
			else if (n >= 72 && n <= 77) {    //Vn_z
				temp[n - 72] = x;
				if (n == 77) {
					v.V_z = str2int(temp, 6, 1)*0.0001;
				}
				n++;
			}
			else if (n >= 78 && n <= 93) {    //径度
				jwg[15 + 78 - n] = x;
				if (n == 93) {
					p.x = str2double(jwg,16);
				}
				n++;
			}
			else if (n >= 94 && n <= 109) {    //纬度
				jwg[15 + 94 - n] = x;
				if (n == 109) {
					p.y = str2double(jwg, 16);
				}
				n++;
			}
			else if (n >= 110 && n <= 117) {    //高度
				jwg[7 + 110 - n] = x;
				if (n == 117) {
					p.z = str2float(jwg, 8);
				}
				n++;
			}
            else if (n >= 118 && n <= 119) {    //标志
				temp[n-118] = x;
                if(n == 119){
                    flag = str2int(temp,2,1);
                }
                n++;
			}
            else if(n >=120 && n <= 121){     //posType
                temp[n-120] = x;
                if(n == 121){
                    posType = str2int(temp,2,1);
                }
                n++;
            }
             else if(n >=122 && n <= 123){     //headType
                temp[n-122] = x;
                if(n == 123){
                    headType = str2int(temp,2,1);
                }
                n++;
            }
            else if (n >= 124 && n <= 139) {    //gps径度
				jwg[15 + 124 - n] = x;
				if (n == 139) {
					gps.lon = str2double(jwg,16);
				}
				n++;
			}
             else if (n >= 140 && n <= 155) {    //gps纬度
				jwg[15 + 140 - n] = x;
				if (n == 155) {
					gps.lat = str2double(jwg,16);
				}
				n++;
			}
             else if (n >= 156 && n <= 171) {    //gps高度
				jwg[15 + 156 - n] = x;
				if (n == 171) {
					gps.alt = str2double(jwg,16);
				}
				n++;
			}
            else if (n >= 172 && n <= 179) {    //gps_VelE
				jwg[7 + 172 - n] = x;
				if (n == 179) {
					gps.V_E = str2float(jwg,8);
				}
				n++;
			}
            else if (n >= 180 && n <= 187) {    //gps_VelN
				jwg[7 + 180 - n] = x;
				if (n == 187) {
					gps.V_N = str2float(jwg,8);
				}
				n++;
			}
            else if (n >= 188 && n <= 195) {    //gps_VelU
				jwg[7 + 188 - n] = x;
				if (n == 195) {
					gps.V_U = str2float(jwg,8);
				}
				n++;
			}
             else if (n >= 196 && n <= 203) {    //gps_yaw
				jwg[7 +196 - n] = x;
				if (n == 203) {
					gps.yaw = str2float(jwg,8);
				}
				n++;
			}
            else if(n >=206 && n <= 209){     //左前轮
                temp[n- 206] = x;
                if(n == 209){
                    car.f_l = str2int(temp,4,1) / 3.6;
                }
                n++;
            }
            else if(n >=210 && n <= 213){     //右前轮
                temp[n- 210] = x;
                if(n == 213){
                    car.f_r = str2int(temp,4,1) / 3.6;
                }
                n++;
            }
            else if(n >=214 && n <= 217){     //左后轮
                temp[n- 214] = x;
                if(n == 217){
                    car.b_l = str2int(temp,4,1) / 3.6;
                }
                n++;
            }
            else if(n >=218 && n <= 221){     //右后轮
                temp[n- 218] = x;
                if(n == 221){
                    car.b_r = str2int(temp,4,1) / 3.6;
                }
                n++;
            }

            else {
                n++;
                if (n >= 239 ) {                    
                    //ros::Time stamp(frame_imu.seconds - 18 + (frame_imu.week * 7 * 24 * 3600) + 315964800);
                    //imu_raw.header.stamp = stamp;
                    imu_raw.header.frame_id = "imu-link";
                    //四元数位姿,所有数据设为固定值，可以自己写代码获取IMU的数据，，然后进行传递
                    imu_raw.orientation.x = 0;
                    imu_raw.orientation.y = 0;
                    imu_raw.orientation.z = 0;
                    imu_raw.orientation.w = 0;
                    //线加速度
                    imu_raw.linear_acceleration.x = a.A_x;
                    imu_raw.linear_acceleration.y = a.A_y;
                    imu_raw.linear_acceleration.z = a.A_z;
                    //角速度
                    imu_raw.angular_velocity.x = g.G_x;
                    imu_raw.angular_velocity.y = g.G_y;
                    imu_raw.angular_velocity.z = g.G_z;

                    inscov.header.stamp = ros::Time::now();
                    inscov.latitude = p.y;
                    inscov.longitude = p.x;
                    inscov.altitude = p.z;
                    inscov.GpsTime = t_time;
                    inscov.E_vel = v.V_x;
                    inscov.N_vel = v.V_y;
                    inscov.U_vel = v.V_z;
                    inscov.pitch = at.Att_x;
                    inscov.roll = at.Att_y;
                    inscov.yaw = at.Att_z;
                    inscov.g_alt = gps.alt;
                    inscov.g_lon = gps.lon;
                    inscov.g_lat = gps.lat;
                    inscov.g_E = gps.V_E;
                    inscov.g_N = gps.V_N;
                    inscov.g_U = gps.V_U;
                    inscov.g_yaw = gps.yaw;
                    inscov.car_f_l = car.f_l;
                    inscov.car_f_r = car.f_r;
                    inscov.car_b_l = car.b_l;
                    inscov.car_b_r = car.b_r;
                    inscov.flag = flag;
                    inscov.posType = posType;
                    inscov.headType = headType;
                    // vx = vx + a.A_x * 0.01;
                    // vy = vy + a.A_y * 0.01;header
                    // vz = vz + a.A_z * 0.01;
                    // yaw = yaw + g.G_z * 0.01;
                    // if(yaw > 180) {
                    //     yaw = yaw - 360;
                    // }
                    // if(yaw < -180){
                    //     yaw = yaw + 360;
                    // }
                    // imu_raw.orientation.x = vx;
                    // imu_raw.orientation.y = vy;
                    // imu_raw.orientation.z = vz;
                    // imu_raw.orientation.w = yaw;

                    std::string temp(reinterpret_cast<const char*>(message));
                    // std::cout<<"message: "<<temp<<std::endl;
                    std::cout<<"i: "<<i<< " n: " << n << " posType: "<<posType<<" headType: "<<headType<<std::endl;
                    // if (posType == 50 && headType == 50 ) {
                    //     imu_raw_pub.publish(imu_raw);
                    //     ins_cov_pub.publish(inscov);
                    // }

                    imu_raw_pub.publish(imu_raw);
                    ins_cov_pub.publish(inscov);
                    start_flag = 0;
                    n = 0;
                }
            }
        }
        i++;
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
    imu_raw_pub = nh.advertise<sensor_msgs::Imu>("/gnss/imu_data", 100);
    ins_raw_pub = nh.advertise<ins_pub::INSPVAXA_Type>("/gnss/ins_data", 10);
    gps_raw_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
    ins_cov_pub = nh.advertise<ins_pub::INSCOV>("/gps/ins_cov", 10);
    ros::Rate loop_rate(100);
    uint8_t temp;
    uint8_t hh = 240;
    uint8_t ll = 15;
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
    ins_pub::INSCOV inscov_info;

    while (ros::ok())
    {
        //reading data
        DataBuffer buffer;
        try
        {
            pRealNposSensor.serial_port.Read(buffer, 120, 5); //串口读取数据
            //printf("\nwating buffer length is %d \n",buffer.size());
         }
         catch (ReadTimeout)
         {
                //printf("\nbuffer length is%d\n",buffer.size());
        }
        unsigned char message[MAX_OUT_SIZE]={0};
        for(int i=0;i<buffer.size();i++){

            temp = buffer[i] & hh;    //high byte
            temp=temp>>4;
            message[2*i] = int2char(temp);

            temp = buffer[i] & ll;   //low byte
            message[2*i + 1] = int2char(temp);


        }
        //memcpy(message,  &buffer[0], buffer.size());
       // pRealNposSensor.BufferIncomingData(message, imu_raw, inspvaxa_info,navsatfix,inscov_info, buffer.size());
        BINARY_LOG_TYPE test;
        pRealNposSensor.ParseBinary(message,240,test, imu_raw, inspvaxa_info, navsatfix,inscov_info);
        //ParseBinary(data_buffer_, buffer_index_, message_id_, imu_raw, inspvaxa_info, navsatfix,inscov);
        ros::spinOnce();
        //loop_rate.sleep();
    }
    return 0;
}
