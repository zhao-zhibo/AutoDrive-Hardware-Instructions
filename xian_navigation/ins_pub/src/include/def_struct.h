int char2int(char c);          //�ַ�ת����
unsigned char int2char(int c);
int str2int(char *p, int len,int u_flag); //�ַ���ת����
double str2double(char *p, int len);   //�ַ���ת������
float  str2float(char *p, int len);   //�ַ���ת������
typedef struct Gyro {    //��������
	double G_x = 0.0;
	double G_y = 0.0;
	double G_z = 0.0;
}Gyro;

typedef struct Acc {    //���ٶȼ�
	double A_x = 0.0;
	double A_y = 0.0;
	double A_z = 0.0;
}Acc;

typedef struct Att {    //Att��̬
	double Att_x = 0.0;
	double Att_y = 0.0;
	double Att_z = 0.0;
}Att;

typedef struct Vn {    //Vn�ٶ�
	double V_x = 0.0;
	double V_y = 0.0;
	double V_z = 0.0;
}Vn;

typedef struct pos {  //��γ��
	double x;   //����
	double y;   //γ��
	double z;   //�߶�
}pos;

typedef struct car_v{
	float f_l;     //左前轮
	float f_r;    //右前轮
	float b_l;   //左后轮
	float b_r;   //右后轮
}car_v;

typedef struct Gps{
	double lat;
	double lon;
	double alt;
	float V_E;
	float V_N;
	float V_U;
	float yaw;
}Gps;