//=============================================================
/* Autonomous Vehicle System Homework - Mobile Robot

Template OpenGL by 
*    Dr. Abdul Muis, MEng.
*    Autonomous Control Electronics (ACONICS) Research Group
*    http://www.ee.ui.ac.id/aconics

*///=============================================================

#include <stdio.h> 
#include <stdlib.h> 
#include <GL/glut.h> // Header File For The GLUT Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library
#include <unistd.h> // Header file for sleeping.
#include <math.h> 
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/time.h>
#include <time.h>
#include "planar.c"

/* ascii code for the escape key */
#define ESCkey	27

/* The number/handle of our GLUT window */
int window, wcam;  
int sensorwindow, backtopwindow, irwindow;

/* To draw a quadric model */
GLUquadricObj *obj;

// ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.3

#define Link1 L1
#define Link2 L2

#define sen8 475
#define sen7 410
#define sen6 346
#define sen5 282
#define sen4 217
#define sen3 153
#define sen2 89
#define sen1 25

//bobot sensor
int ir8=0, ir7=0, ir6=0, ir5=0, ir4=0, ir3=0, ir2=0, ir1=0;

//Variables untuk tugas animasi (animate 1 2 dan 3. Bukan mengikuti garis)
float *tetha1 = &q1;
float *tetha2 = &q2;
float *tetha3 = &q3;

float *translasi1 = &ts1;
float *translasi2 = &ts2;
float *translasi3 = &ts3;

// Variables untuk mengikuti garis

float *rx=&px; //posisi mobile robot di sumbu x
float *ry=&py; //posisi mobile robot di sumbu y
float *shi=&absRot; //sudut yg menunjukkan arah hadap mobile robot. Diguanakn untuk update posisi di sumbu x dan y

float *ql = &q_left; //sudut roda kiri
float *qr = &q_right; //sudut roda kanan

float *ql_ref = &q_left_ref;
float *qr_ref = &q_right_ref;

float ql_ref_old, qr_ref_old;
float omega_l_ref, omega_r_ref;

float sensor, sensor_old, dt, current_time, error_dot;
float error_prev, error_prev1, error_prev2;
float integral_error, integral_error_old;
float Kp, Ki, Kd;
float control;

float dt1, current_time1;
float omega_l_old, omega_r_old;
float error_left, error_old_left, integral_error_left, integral_error_old_left, error_dot_left;
float error_right, error_old_right, integral_error_right, integral_error_old_right, error_dot_right;
float Kpm, Kim, Kdm;
float control_left, control_right;

float modulator;
float omega_l, omega_r;
float *pwm_l = &pwm_left; 
float *pwm_r = &pwm_right;

/// Variables untuk visualisasi kamera
#define img_height 100
#define img_width 500
#define floor_height 2000
#define floor_width 2000

#define panjang 0.145

unsigned char gambarGray[img_height+2][img_width+2];
unsigned char gambarR[img_height+2][img_width+2];
unsigned char gambarG[img_height+2][img_width+2];
unsigned char image_raw[img_height+1][img_width+1];
void camera_sensor(void);
FILE *fileimage;
unsigned char* data = NULL;
unsigned int textureNumber;

char debug=0;


void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display_main(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal
void display_backtop(void);
void display_sensor(void);
void disp_robot(void);

//Function newx dan newy untuk update posisi mobile robot
float newx(float x, float y){
  return *rx + x*cos(*shi) - y*sin(*shi);
}
float newy(float x, float y){
  return *ry + x*sin(*shi) + y*cos(*shi);
}

/* define color */ 
GLfloat red[4] = {1.0,0.0,0.0,1.0};
GLfloat green1[4]  ={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  ={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  ={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  ={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4] ={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4]={0.8, 0.8, 0.0, 1.0};
GLfloat abu2[4]={0.5,0.5,0.5,1.0};
GLfloat gray1[4]  ={0.1, 0.1, 0.1, 1.0};
GLfloat gray2[4]  ={0.2, 0.2, 0.2, 1.0};
GLfloat gray3[4]  ={0.3, 0.3, 0.3, 1.0};
GLfloat gray4[4]  ={0.4, 0.4, 0.4, 1.0};
GLfloat gray5[4]  ={0.5, 0.5, 0.5, 1.0};
GLfloat gray6[4]  ={0.6, 0.6, 0.6, 1.0};
GLfloat gray7[4]  ={0.7, 0.7, 0.7, 1.0};
GLfloat gray8[4]  ={0.8, 0.8, 0.7, 1.0};
GLfloat gray9[4]  ={0.9, 0.9, 0.7, 1.0};


void  drawOneLine(double x1, double y1, double x2, double y2) 
   {glBegin(GL_LINES); glVertex3f((x1),(y1),0.0); glVertex3f((x2),(y2),0.0); glEnd();}
   
void  model_cylinder(GLUquadricObj * object, GLdouble lowerRadius,
  GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2)
{
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void  model_box(GLfloat width, GLfloat depth, GLfloat height, GLfloat *color1, GLfloat *color2, GLfloat *color3, int color)
{
   width=width/2.0;depth=depth/2.0;height=height/2.0;
   glBegin(GL_QUADS);
// top
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth, height);
    glVertex3f( width,-depth, height);
    glVertex3f( width, depth, height);
    glVertex3f(-width, depth, height);
   glEnd();
   glBegin(GL_QUADS);
// bottom
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth,-height);
    glVertex3f( width,-depth,-height);
    glVertex3f( width, depth,-height);
    glVertex3f(-width, depth,-height);
   glEnd();
   glBegin(GL_QUAD_STRIP);
// sides
    if (color==1) 
	    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    glVertex3f(-width,-depth,height);
    glVertex3f(-width,-depth,-height);
    glVertex3f(width,-depth,height);
    glVertex3f(width,-depth,-height);
    glVertex3f(width,depth,height);
    glVertex3f(width,depth,-height);
    glVertex3f(-width,depth,height);
    glVertex3f(-width,depth,-height);
    glVertex3f(-width,-depth,height);
   glEnd();
}



void disp_floor(int grid)
{
  int i,j,flagc=1;

   if (grid){
      glPushMatrix();
  
      GLfloat dx=4.5,dy=4.5;
      GLint amount=15;
      GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount, y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
      for(i = 0; i<=48; i++){
         drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
         drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
      }

      glPopMatrix();
   }

   glPushMatrix();
   glEnable(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, textureNumber);
    glColor3f(0.0f,0.0f,0.0f);
    glBegin(GL_POLYGON); // three
    // urutan koordinate bisa membuat gambar terotasi / terputar
    glTexCoord2f(0,1); glVertex3f(-1.0f, -1.0f, 0);//glVertex3f(-1.0f, 1.0f, 0);
    glTexCoord2f(0,0); glVertex3f(-1.0f,1.0f, 0);//glVertex3f(-1.0f,-1.0f, 0);
    glTexCoord2f(1,0); glVertex3f( 1.0f,1.0f, 0);//glVertex3f( 1.0f,-1.0f, 0);
    glTexCoord2f(1,1); glVertex3f( 1.0f,-1.0f, 0);//glVertex3f( 1.0f, 1.0f, 0);
    glEnd();

   glDisable(GL_TEXTURE_2D);
  glPopMatrix();

}

void  lighting(void)
{

	GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
	GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
	GLfloat light_specular[] = {0.3, 0.3, 0.3, 1.0};
	GLfloat light_position[] = {2, 0.1, 7,1.0};
	GLfloat spot_direction[] = {0.0, -0.1, -1.0, 1.0};

	glClearColor(0.0, 0.0, 0.0, 0.0);     
  
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void keyboard(unsigned char key, int i, int j)
{
   switch(key){
      case ESCkey: exit(1); break;
      case 'w': *rx=newx(0.1, 0); *ry=newy(0.1, 0); break; 
      case 's': *rx=newx(-0.1, 0); *ry=newy(-0.1, 0); break; 
      case 'd': *ql=*ql+(5*DTR); *qr=*qr-(5*DTR); break;
      case 'a': *ql=*ql-(5*DTR); *qr=*qr+(5*DTR); break;

      // case 'd': *shi=*shi-(5*DTR); break;
      // case 'a': *shi=*shi+(5*DTR); break;

      case 'u': glTranslatef(0.1,0,0); break;
      case 'U': glTranslatef(-0.1,0,0); break;
      case 'i': glTranslatef(0,0.1,0); break;
      case 'I': glTranslatef(0,-0.1,0); break;
      case 'o': glTranslatef(0,0,0.1); break;
      case 'O': glTranslatef(0,0,-0.1); break;
      case 'j': glRotatef(10,1,0,0); break;
      case 'J': glRotatef(-10,1,0,0); break;
      case 'k': glRotatef(10,0,1,0); break;
      case 'K': glRotatef(-10,0,1,0); break;
      case 'l': glRotatef(10,0,0,1); break;
      case 'L': glRotatef(-10,0,0,1); break;
      
   }
}

// Draw Object
void display_main(void)
{
//   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   //glLoadIdentity();  // Reset View
   disp_floor(1);
   
   disp_robot();

   /* since window is double buffered, 
      Swap the front and back buffers (used in double buffering). */
   glutSwapBuffers() ; 
}

void display_ir(void)
{
   glClear(GL_COLOR_BUFFER_BIT);
   glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, image_raw);
   glutSwapBuffers();

}

void Sim_main(void)
{
	unsigned long Xr=0,Yr=0, Xg=0,Yg=0, Xb=0,Yb=0; // titik untuk menghitung sum
	int Nr=0, Ng=0, Nb=0;
	static unsigned int Rx,Ry, Gx,Gy, Bx,By; // untuk menyimpan hasil titik berat
	unsigned int i,j,k;
  static int count=0;
  glutSetWindow(window);
  
  //animate(count);
  //animate2();
  //animate3();
  //animate4();
  animated();

  display_main();
  // Retrieve_serial();
   /*ini yg sebelum tugas 2 kamera
   glutSetWindow(wcam);
   camera_sensor();
   */

   glutSetWindow(backtopwindow);
  display_backtop();
  glutSetWindow(sensorwindow);
  display_sensor();
  glutSetWindow(irwindow);
  display_ir();

  usleep(2000);
}

void disp_robot(void)
{
   
   glPushMatrix();
    glTranslatef(*rx,*ry,0.025);
    glRotatef(*shi*RTD,0,0,1);
    model_box(0.05, (0.09-0.005)*2, 0.01, gray8, gray7, gray6,1);
    glPushMatrix();
       glTranslatef(0,0.09,0);
       glRotatef(-90,1,0,0);
       model_cylinder(obj, 0.025/2, 0.025/2, 0.01, 2, blue3,blue3);
    glPopMatrix();
    glPushMatrix();
       glTranslatef(0,-0.09,0);
       glRotatef(90,1,0,0);
       model_cylinder(obj, 0.025/2, 0.025/2, 0.01, 2, blue3,blue3);
    glPopMatrix();
    glPushMatrix();
       glTranslatef(0.075,0,0);
       model_box(0.15, 0.01, 0.01, gray8, gray7, gray6,1);
     
    glPopMatrix();
    glPushMatrix();
      glTranslatef(0.15,0,0);
      model_box(0.03, 0.1, 0.01, red,red,red,1);
     
    glPopMatrix();
    glPopMatrix();

}

void display_sensor(void)
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   float floor_x=0.145+0.005, floor_y=0, floor_z=0;
   float sense_x=0.145, sense_y=0, sense_z=0.2;

   float floor_x_new = newx(floor_x, floor_y);
   float floor_y_new = newy(floor_x, floor_y);
   float sense_x_new = newx(sense_x, sense_y);
   float sense_y_new = newy(sense_x, sense_y);

   // gluPerspective(6.34, 5, 0.19, 1);
   glFrustum(-0.05,0.05,0.01,-0.01,0.19,1);
   gluLookAt(sense_x_new, sense_y_new, sense_z, floor_x_new, floor_y_new,floor_z, 0.0, 0.0, 1.0); 

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   disp_floor(0);
   disp_robot();
   lighting();
   glShadeModel(GL_SMOOTH) ; 
   glutSwapBuffers();  
   // Convert camera view to image
    // Set Luminance Value to be 1 (max)
    glPixelTransferf(GL_RED_SCALE,0.3333*0.2);
    glPixelTransferf(GL_GREEN_SCALE,0.3334*0.2);
    glPixelTransferf(GL_BLUE_SCALE,0.3333*0.2); 

    glReadPixels(0,0, img_width,img_height, GL_LUMINANCE,GL_UNSIGNED_BYTE, image_raw);

    ir8 = (image_raw[50][sen8]<50) ? 1:0;
   ir7 = (image_raw[50][sen7]<50) ? 1:0;
   ir6 = (image_raw[50][sen6]<50) ? 1:0;
   ir5 = (image_raw[50][sen5]<50) ? 1:0;
   ir4 = (image_raw[50][sen4]<50) ? 1:0;
   ir3 = (image_raw[50][sen3]<50) ? 1:0;
   ir2 = (image_raw[50][sen2]<50) ? 1:0;
   ir1 = (image_raw[50][sen1]<50) ? 1:0;
   
   image_raw[50][sen8] = ir8*255;
   image_raw[50][sen7] = ir7*255;
   image_raw[50][sen6] = ir6*255;
   image_raw[50][sen5] = ir5*255;
   image_raw[50][sen4] = ir4*255;
   image_raw[50][sen3] = ir3*255;
   image_raw[50][sen2] = ir2*255;
   image_raw[50][sen1] = ir1*255;

   Kp = 5.0;
   Kd = 0.0;

   sensor = (float)(ir8*(-4.0) + ir7*(-3.0) + ir6*(-2.0) + ir5*(-1.0) + ir4*1.0 + ir3*2.0 + ir2*3.0 + ir1*4.0);
   dt = (double)((clock() - current_time)/1000000);

   if(dt!=0.0){
      control = (Kp*sensor + Kd*(sensor - sensor_old)/dt)*DTR;
      *ql += -control + 0.2;
      *qr += control + 0.2;
      if(sensor == 0){
   	   *ql += -control + 0.2;
   	   *qr += control + 0.2;
	   }
      if(sensor < 0){
   	   *ql += -control +0.2;
	   }
      if(sensor > 0){
   	   *qr += control + 0.2;
	   }
	   sensor_old = sensor;
      integral_error_old = integral_error;
      dt1 = (double)((clock() - current_time1)/1000000);
      if(dt1 != 0.0){
         modulator = (double)(1/5) * fmod(clock(), 5);
         if(fabs(control_left)/255.0 >= modulator){
            *pwm_l = 5.0;
         }
         else{
            *pwm_l = 0.0;
         } 
         if(omega_l_ref < 0){
            *pwm_l = -5.0;
         }
         if(fabs(control_right)/255.0 >= modulator){
            *pwm_r = 5.0;
         }
         else{
            *pwm_r = 0.0;
         }
         if(omega_r_ref < 0){
            *pwm_r = -5.0;
         }
         omega_l = (200*dt1/(1+300*dt1))*(*pwm_l);
         omega_r = (200*dt1/(1+300*dt1))*(*pwm_r);	
         *ql += omega_l*dt;
         *qr += omega_r*dt;
      }

   current_time = clock();
   current_time1 = clock();

}
}

// void camera_sensor(void)
// {
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    disp_floor();
//    disp_robot();
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();

//    //Cari posisi end effector (bisa dengan forward kinematic)
//    float sense_x=0.145, sense_y=0, sense_z=0.5;
//    float floor_x=0.145, floor_y=0, floor_z=0.2;
//    // Frustum --> Left, Right, Bottom, Top, Near, Far (focus->near)
//    glFrustum(-0.00025*img_width, 0.00025*img_width, -0.00025*img_height, 0.00025*img_height, 0.1, 8.0);
//    gluLookAt(sense_x+*translasi1,sense_y+*translasi2,sense_z+*translasi3,floor_x+*translasi1,floor_y+*translasi2,floor_z+*translasi3,-(*tetha3*RTD)/90,1+(*tetha3*RTD)/90,0);
//    //gluLookAt(sense_x, sense_y, sense_z, floor_x, floor_y, floor_z, 1,0,0); default gluLookAt
   
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();

//    glutSwapBuffers();
//    //convert camera view to image
//    // set Luminance Value to be 1 (max)
//    glPixelTransferf(GL_RED_SCALE, 0.3333);
//    glPixelTransferf(GL_GREEN_SCALE, 0.3334);
//    glPixelTransferf(GL_BLUE_SCALE, 0.3333);

//    time_t timestart=0;
//    time_t timeend = 0;

//    glReadPixels(0,0, img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, image_raw);

// }

void camera_result(void)
{
   glClear(GL_COLOR_BUFFER_BIT);
   if (debug)
      glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, gambarG);
   else
      glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, gambarGray);

   glutSwapBuffers();
}



void camera_window(void)
{
   /*---------------Camera Window------------*/

   glutInitWindowSize(img_width, img_height);
   glutInitWindowPosition (500, 100);
   sensorwindow = glutCreateWindow("Camera Process");
   glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
   glutDisplayFunc(&display_sensor);
   glutKeyboardFunc(&keyboard);
}

void ir_window(void)
{
  /*----------Camera Window----------*/
    //glutInitDisplayMode(GLUT_DOUBLE |  GLUT_DEPTH);

   glutInitWindowSize(img_width,img_height);    
   glutInitWindowPosition (500, 100);
   irwindow = glutCreateWindow("Tampak Sensor");
   glClearColor(0.0f, 0.0f, 0.0f, 1.0f); 
   glutDisplayFunc (&display_ir) ;
   glutKeyboardFunc(&keyboard);
}

void display_backtop(void){

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, 1.25, 0.01, 2);
  float sense_x=-0.2, sense_y=0, sense_z=0.3;
  float floor_x=0.3, floor_y=0.0, floor_z=0.0;

  float floor_x_new = newx(floor_x, floor_y);
   float floor_y_new = newy(floor_x, floor_y);
   float sense_x_new = newx(sense_x, sense_y);
   float sense_y_new = newy(sense_x, sense_y);

   gluLookAt(sense_x_new, sense_y_new, sense_z, floor_x_new, floor_y_new,floor_z, 0.0, 0.0, 1.0); 
  glMatrixMode(GL_MODELVIEW);
  lighting();
  disp_floor(0);
  disp_robot();
  glutSwapBuffers(); 
}

void camera_backtopwindow(void)
{
  /*----------Camera Window----------*/
    // glutInitDisplayMode(GLUT_DOUBLE |  GLUT_DEPTH | GLUT_RGB);
   glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH);
   glutInitWindowSize(500,300);    
   glutInitWindowPosition (500, 400);
   backtopwindow = glutCreateWindow("Camera Back Top");
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glutDisplayFunc(&display_backtop) ;
   glutKeyboardFunc(&keyboard);
}

void main_window(void) 
{ 
   glutInitWindowSize(800,400); 
   glutInitWindowPosition (40, 100);

   /* Open a window */  
   window = glutCreateWindow ("Simple Window");
   /* Clear background to (Red, Green, Blue, Alpha) */
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glEnable(GL_DEPTH_TEST); // Enables Depth Testing
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(60.0, 2, 0.2, 8);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0.2, -1.0, 1.5,  0.0, 0.2, 0.2,  0.0, 0.0, 1.0); 
   lighting();
   
   /* When the shading model is GL_FLAT only one colour per polygon is used, 
      whereas when the shading model is set to GL_SMOOTH the colour of 
      a polygon is interpolated among the colours of its vertices.  */
   glShadeModel(GL_SMOOTH) ; 

   glutDisplayFunc (&display_main) ;
   glutKeyboardFunc(&keyboard);

}

int loadGLTexture(const char *filename, int width, int height){
   // open texture data
     free(data);
        
     // data = glmReadPPM(filename, &width, &height);

     // Pastikan ukuran file tidak besar hanya 500x500
     fileimage = fopen(filename,"r");
     if (fileimage == NULL) return 0;
 
     // allocate buffer
     data = (unsigned char*) malloc(width * height * 3);
 
     //read texture data
     size_t assa = fread(data, width * height * 3, 1, fileimage);
     fclose(fileimage);

     // for(int i=0;i<width * height * 3;i++)
     //   if (data[i]<255) printf("%d ",data[i]);

     unsigned int textureID;
     int border=0;
     int depth=width * height * 3;
     glGenTextures(1, &textureID);

     glBindTexture( GL_TEXTURE_2D, textureID);
     // //texture colors should replace the original color values
     glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,GL_REPLACE ); //GL_MODULATE mengikuti warna dasar
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR );
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER );
     glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,GL_CLAMP_TO_BORDER );
    

    // Cara #1
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

    // Cara #2
    // gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height,GL_RGB, GL_UNSIGNED_BYTE, data );

    // 3D texture
     //glEnable(GL_TEXTURE_3D);
     // glBindTexture(GL_TEXTURE_3D, textureID);
     // glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
     // glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
     // glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
     // GLfloat borderColor[4] = {1.0f,1.0f,1.0f,1.0f};
     // glTexParameterfv(GL_TEXTURE_3D, GL_TEXTURE_BORDER_COLOR, borderColor);
     // //define how to filter the texture
     // glTexParameteri (GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
     // glTexParameteri (GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
     // //texture colors should replace the original color values
     // glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //GL_MODULATE
     // // specify the 2D texture map
     // glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB, width, height, depth, border, GL_RGB, GL_UNSIGNED_BYTE, data);
     // return unique texture identifier
     return textureID;
}


// void init(void) 
// { 
//    obj = gluNewQuadric(); 
//    /* Clear background to (Red, Green, Blue, Alpha) */
//    glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
//    glEnable(GL_DEPTH_TEST); // Enables Depth Testing
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    gluPerspective(60.0, 2, 0.2, 8);
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//    gluLookAt(0.2, -1.0, 1.5,  0.0, 0.2, 0.2,  0.0, 0.0, 1.0); 
// 	 lighting();
	 
//    /* When the shading model is GL_FLAT only one colour per polygon is used, 
//       whereas when the shading model is set to GL_SMOOTH the colour of 
//       a polygon is interpolated among the colours of its vertices.  */
//    glShadeModel(GL_SMOOTH) ; 

//    glutDisplayFunc (&display) ;
//    glutKeyboardFunc(&keyboard);
// } 

// Main Program
int main(int argc, char** argv)
{

 // Initialize GLUT
   /* Initialize GLUT state - glut will take any command line arguments 
      see summary on OpenGL Summary */  
   glutInit (&argc, argv);
   
   // Berikut jika ingin menggunakan serial port
   //fd = open_port();
   //init_port(fd);

   /* Select type of Display mode:   
      Double buffer 
      RGBA color
      Alpha components supported 
      Depth buffer */  
   //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
   /* set a 400 (width) x 400 (height) window and its position */
   

   obj = gluNewQuadric(); 
   /* Initialize our window. */
   init_robot();
   main_window() ;
   textureNumber = loadGLTexture("track3.ppm",500,500);
   camera_backtopwindow();
   textureNumber = loadGLTexture("track3.ppm",500,500);
   camera_window(); 
   textureNumber = loadGLTexture("track3.ppm",500,500);
   ir_window(); 

   /* Register the function to do all our OpenGL drawing. */
   glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

   /* Start Event Processing Engine */ 
   glutMainLoop () ;
   return 0 ;
   // glutInitWindowSize(800,400);	
   // glutInitWindowPosition (40, 100);

   // /* Open a window */  
   // window = glutCreateWindow ("Simple Window");

   // /* Initialize our window. */
   // init() ;
   // init_robot();
   // camera_window();

   // /* Register the function to do all our OpenGL drawing. */
   // glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

   // /* Start Event Processing Engine */ 
   // glutMainLoop () ;
   // return 0 ;
}           