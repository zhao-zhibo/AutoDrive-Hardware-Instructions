#include "def_struct.h"
#include "math.h"
int char2int(char c) {       
	int num;
	switch (c) {
	case '0':num = 0; break;
	case '1':num = 1; break;
	case '2':num = 2; break;
	case '3':num = 3; break;
	case '4':num = 4; break;
	case '5':num = 5; break;
	case '6':num = 6; break;
	case '7':num = 7; break;
	case '8':num = 8; break;
	case '9':num = 9; break;
	case 'A':num = 10; break;
	case 'B':num = 11; break;
	case 'C':num = 12; break;
	case 'D':num = 13; break;
	case 'E':num = 14; break;
	case 'F':num = 15; break;
	default:num = 0;
	}
	return num;
}
unsigned char int2char(int c){

    switch(c){
        case 0:return '0';break;
        case 1:return '1';break;
        case 2:return '2';break;
        case 3:return '3';break;
        case 4:return '4';break;
        case 5:return '5';break;
        case 6:return '6';break;
        case 7:return '7';break;
        case 8:return '8';break;
        case 9:return '9';break;
        case 10:return 'A';break;
        case 11:return 'B';break;
        case 12:return 'C';break;
        case 13:return 'D';break;
        case 14:return 'E';break;
        case 15:return 'F';break;
        default:
            return '0';

    }
}

int str2int(char *p, int len,int u_flag) {
	int n, num = 0;
	for (n = 0; n < len; n++) {
		num += char2int(p[n])*pow(16, (len - n - 1));
	}
	if (u_flag == 1) {
		return (char2int(p[0]) < 8) ? num : (num - pow(16, len));
	}
	else {
		return num;
	}
}

double str2double(char *p, int len)
{

	double res;
	int a, b;
	unsigned char buff[8];
	for (int i = 0; i < 8; i++) {
		a = char2int(p[i * 2 + 1]) * 16;
		b = char2int(p[i * 2]);
		buff[i] = char(a + b);
	}

	double* reslut = (double*)buff;
	return (*reslut);
}

float  str2float(char *p, int len)
{
	double res;
	int a, b;
	unsigned char buff[4];
	for (int i = 0; i < 4; i++) {
		a = char2int(p[i * 2 + 1]) * 16;
		b = char2int(p[i * 2]);
		buff[i] = char(a + b);
	}

	float* reslut = (float*)buff;
	return (*reslut);
}