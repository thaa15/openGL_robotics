#include <math.h>
// #include "serial.h"
#define PI		3.14159265358
#define DTR 	PI/180.0				   // Conversi degree to radian
#define RTD 	180.0/PI				   // Conversi degree to radian

#define L1	0.3   // link1
#define L2	0.2   // link2

float q1;
float q2;
float ex,ey,x_d,y_d,t,x,y,ddx,ddy,dq1,dq2,ddq1_ref,ddq2_ref,v1,v2,y_cmd,x_cmd;
float ex_old,ey_old;
float objx=0.3;
float objy=0.5;
float x_init = 0.3, y_init = 0.5;

void init_robot()
{
	q1=0.0 * DTR;
	q2=30.0 * DTR;
	x_cmd = 0.0;
	y_cmd = 0.0;
}

// void Retrieve_serial(void) {
//   int retval=1, i,j,k,l;

//   unsigned char sdata[3]; 
//   unsigned char baca;
  
  
// 	i=1;

//   while (i>0) {
//     fcntl(fd, F_SETFL, FNDELAY); 
//     i=read(fd, &baca, 1);
//     if ((i==1) && (baca == 0xF5)) {
//     	printf("masuk\n");
//     	sdata[0]=baca;
//     	while (i<3) {
//     		  if (read(fd, &baca, 1)>0) {sdata[i]=baca; i++;}
//     	}
//    	  printf("terbaca %x  %x  %x \n",sdata[0],sdata[1],sdata[2]);
//    	  q1=(sdata[1])*180.0/255.0*DTR;
//    	  q2=(sdata[2])*180.0/255.0*DTR;
//     }
//   } 

// }
