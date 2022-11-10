
//variabel2, fungsi, animasi

#include <math.h>
#include <stdio.h>
#define PI		3.14159265358
#define DTR 	PI/180.0				   // Conversi degree to radian
#define RTD 	180.0/PI				   // Conversi radian to degree

#define L1	0.3   // link1
#define L2	0.2   // link2



float objx=0.3;
float objy=0.5;

//variable yang dimasukkin ke opengl sehingga bisa dianimasiin atau diubah variabelnya

float q1; //untuk rotasi
float q2;
float q3;

float ts1; //untuk translasi
float ts2;
float ts3;

float posx,posy,absRot;

float px, py, absRot_old; 
float dx, dy, vel;
float q_left, q_right;
float q_left_old, q_right_old;
float q_left_ref, q_right_ref;

float pwm_left, pwm_right;

float dq_left, dq_right;

void init_robot()
{
	q1=0.0 * DTR;
	q2=0.0 * DTR;
	q3=0.0 * DTR;

	ts1 = 0.0;
	ts2 = 0.0;
	ts3 = 0.0;
}

float newx_planar(float x, float y){
  return posx + x*cos(absRot) - y*sin(absRot);
}
float newy_planar(float x, float y){
  return posy + x*sin(absRot) + y*cos(absRot);
}

void animated(){
	//Line follower advanced

	dq_left = (q_left - q_left_old)/0.001;
	dq_right =  (q_right - q_right_old)/0.001;
	
	absRot = (0.025/0.18)*(q_right-q_left);
	dx = (0.025/2.0)*cos(absRot)*(dq_right+dq_left);
	dy = (0.025/2.0)*sin(absRot)*(dq_right+dq_left);
	vel = dx*cos(absRot) + dy*sin(absRot);
	
	px = px + vel*cos((absRot_old+absRot)/2.0)*0.001;
	py = py + vel*sin((absRot_old+absRot)/2.0)*0.001;
	
	absRot_old = absRot;
	q_left_old = q_left;
	q_right_old = q_right;
	usleep(100000);
}

void resetloc(){
	q1=0.0 * DTR;
	q2=00.0 * DTR;
	q3=00.0 * DTR;
	ts1=0.0 * DTR;
	ts2=00.0 * DTR;
	ts3=0.0 * DTR;
}

/*
void animate2() {
	//right-forward angle 30 degrees 1 m
	if (rotasi2 >= -30*DTR) rotasi2 -= 5*DTR;
	else if (rotasi2 <= -30*DTR && translasi3 <= 1) translasi3 += 0.05;
}

void animate3(){
	//right forward on slope radius 2 m
	if (translasi4 <=2 ) translasi4 += 2;
	if (translasi4 >= 2 && translasi5 >= -2) translasi5 -= 2;
	else if (translasi4 >= 2 && translasi5 <= -2 && rotasi3 >= -90*DTR) rotasi3 -= 0.1*DTR;
}
*/