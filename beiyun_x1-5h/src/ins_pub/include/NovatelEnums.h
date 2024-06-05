#ifndef NOVATELENUMS
#define NOVATELENUMS
#include <stdint.h>
#ifdef _MSC_VER // using MSVC
    #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
    #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif
namespace navicore {

#define MAX_OUT_SIZE 8192

#define NOVATEL_SYNC_BYTE_1 0xAA
#define NOVATEL_SYNC_BYTE_2 0x44
#define NOVATEL_SYNC_BYTE_3 0x12
#define NOVATEL_SYNC_BYTE_4 0x13

enum BINARY_LOG_TYPE
{
    INSPVASA_LOG_TYPE = 508,
    RAWIMUSA_LOG_TYPE = 325,
    //RAWIMUSA_LOG_TYPE=325,
    RANGEA_LOG_TYPE=43,
    INSPVAXA_LOG_TYPE =1465,
    RAWIMUA_LOG_TYPE=268
};
typedef enum BINARY_LOG_TYPE BINARY_LOG_TYPE;
enum TIME_STATUS
{
    Unknown,Fine
};
enum INS_STATUS
{
    INS_INACTIVE = 0,
    INS_ALIGNING =1,
    INS_HIGH_VARIANCE = 2,
    INS_SOLUTION_GOOD =3,
    INS_SOLUTION_FREE =6,
    INS_ALIGNMENT_COMPLETN =7,
    DETERMINING_ORIENTATION =8,
    WAITING_INITIALPOS = 9,
    WAITING_AZIMUTH =10,
    INITIALIZING_BIASES=11,
    MOTION_DETECT=12
};
enum POS_TYPE
{
    NONE =0,
    FIXEDPOS =1,
    FIXEDHEIGHT =2,
    SINGLE = 16,
    PSRDIFF =17,
    PROPAGATED =19,
    L1_FLOAT =34,
    L1_INT =48,
    RTK_DIRECT_INS = 51,
    INS_PSRSP = 53,
    INS_PSRDIFF =54,
    INS_RTKFLOAT =55,
    INS_RTKFIXED =56
};
typedef struct tagbynav_header_t
{
    unsigned char sync_bytes[3], header_len,port_addr;
    unsigned short msg_ID;
    unsigned char msg_type;
    unsigned short msg_len, sequence;
    unsigned char idle_time;

    // this is actually an enum:
    unsigned char time_status;

    unsigned short week;
    int32_t ms, rcvr_stat;
    unsigned short reserved, version;
}bynav_header_t;

typedef struct tagbyINSPVAXA{
    bynav_header_t header;
    uint32_t slo_status,pos_type;
    double latitude,longitude,altitude;
    float undulation;
    double veL_N,vel_E,vel_U;
    double roll,pitch,azimuth;
    float lat_std,lon_std,alt_std;
    float vN_std,vE_std,vU_std;
    float roll_std,pitch_std,azimuth_std;
    unsigned int ext_sol_stat;
    unsigned short update_time;
    uint32_t checksum;
}ByINSPVAXA;

typedef struct tagoem7_short_header_t
{
    unsigned char sync_bytes[3];
    unsigned char message_len;
    unsigned short msg_ID;
    unsigned short week;
    // unsigned char port_address;
    uint32_t ms;

}oem7_short_header_t;


typedef struct tagINSPVAXA{
    oem7_short_header_t header;
    unsigned short week;
    double  ms;
    // uint32_t slo_status,pos_type;
    double latitude,longitude,altitude;
    // float undulation;
    double veL_N,vel_E,vel_U;
    double roll,pitch,azimuth;
    // float lat_std,lon_std,alt_std;
    // float vN_std,vE_std,vU_std;
    // float roll_std,pitch_std,azimuth_std;
    unsigned int ext_sol_stat;
    // unsigned short update_time;
    uint32_t checksum;
}INSPVAXA;

typedef struct tagRAWIMUSA{
    oem7_short_header_t header;
    // oem7_header_t header;
    uint32_t week;
    double seconds;
    uint32_t IMU_s;
    int32_t z_accel, y_accel, x_accel,
            z_gyro,  y_gyro,  x_gyro;
    // actually an enum
    // uint32_t  checksum;
}RAWIMUSA;

typedef struct tagRAWIMUA{
    bynav_header_t header;
    uint32_t week;
    double seconds;
    uint32_t imu_status;
    int32_t z_accel, y_accel, x_accel,
            z_gyro,  y_gyro,  x_gyro;
    // actually an enum
    uint32_t  checksum;
}RAWIMUA;

typedef struct tagUnionStruct{
    int data_type;//0表示惯导，1表示GNSS
    int week;
    double second;
    double rpy_r,rpy_p,rpy_y;
    double VelN,VelE,VelD;
    double accx_lat,accy_lon,accz_altitu;//accx,accy,acc或者lat,lon,altitu    
    double gyrox_latstd,gyroy_lonstd,gyroz_altitustd;//gyrox,gyroy,gyroz或者lat_std,lon_std,altitu_std
    
}UnionStruct;

}

#endif // NOVATELENUMS

