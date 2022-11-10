#include <stdio.h> 
#include <stdlib.h> 
#include <GL/glut.h> // Header File For The GLUT Library
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library
#include <unistd.h> // Header file for sleeping.
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/time.h>
#include <math.h>
#define PI            3.14159265358
#define DTR           PI/180
#define RTD           180/PI
#define ESCkey        27
#define img_height    100
#define img_width     500
#define floor_height  2000
#define floor_width   2000
#define panjang       0.145
#define sen8          475
#define sen7          410
#define sen6          346
#define sen5          282
#define sen4          217
#define sen3          153
#define sen2          89
#define sen1          25

GLUquadricObj *obj;
// Warna
GLfloat red[4] = {1.0,0.0,0.0,1.0};
GLfloat black[4] = {0.0,0.0,0.0,1.0};
GLfloat white[4] = {1.0,1.0,1.0,1.0};
GLfloat green1[4]  ={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  ={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  ={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  ={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4]  ={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4] ={0.8, 0.8, 0.0, 1.0};
GLfloat abu2[4]   ={0.5,0.5,0.5,1.0};
GLfloat gray1[4]  ={0.1, 0.1, 0.1, 1.0};
GLfloat gray2[4]  ={0.2, 0.2, 0.2, 1.0};
GLfloat gray3[4]  ={0.3, 0.3, 0.3, 1.0};
GLfloat gray4[4]  ={0.4, 0.4, 0.4, 1.0};
GLfloat gray5[4]  ={0.5, 0.5, 0.5, 1.0};
GLfloat gray6[4]  ={0.6, 0.6, 0.6, 1.0};
GLfloat gray7[4]  ={0.7, 0.7, 0.7, 1.0};
GLfloat gray8[4]  ={0.8, 0.8, 0.7, 1.0};
GLfloat gray9[4]  ={0.9, 0.9, 0.7, 1.0};

int window, sensorwindow, backtopwindow, irwindow;
unsigned char image_raw[img_height+1][img_width+1];
FILE *fileimage = NULL;
size_t data_file;
unsigned char *data = NULL;
unsigned int textureNumber;
int ir8=0, ir7 = 0,ir6 = 0, ir5 = 0,ir4 = 0, ir3 = 0,ir2 = 0,ir1 = 0,Kp=12;
float shi=0.0*DTR,rx=0.0,ry=0.0,q1=0.0,q2=0.0,shi_old=0,dx=0,dy=0,dshi=0,dq2=0,dq1=0,rv=0;

void camera_sensor();
void display_main();
void display_backtop();
void display_sensor();
void display_ir();
void display(void); // display robot dan workspace (terdapat fungsi disp) 
void arrowKeyPress(int, int, int);
void init_robot(); // Inisialisasi Join
void keyboard(unsigned char, int , int); // onKeyPress be like
void drawOneLine(double, double, double, double); // Gambar satu garis dengan GL_LINES
void model_cylinder(GLUquadricObj *, GLdouble, GLdouble, GLdouble, GLint, GLfloat *, GLfloat *); // Gambar tabung
void model_box(GLfloat, GLfloat, GLfloat, GLfloat *, GLfloat *, GLfloat *, int); // Gambar Kotak
void disp_floor(int); // Visualisasi workspace
void lighting(void); // Memberikan warna
void disp_robot(); // Menampilkan robot
void simulation(void); // Simulasi robot selama iddle
void init(void); // Mulai
int loadGLTexture(const char *, int, int);
void main_window();
void camera_window();

float newx(float x,float y){
  return rx + x*cos(shi) - y*sin(shi);
}

float newy(float x, float y){
  return ry + x*sin(shi) - y*cos(shi);
}

void main_window(){
  glutInitWindowSize(800,400);	
  glutInitWindowPosition (40, 100);
  window = glutCreateWindow("Thariq Hadyan");

  glClearColor(0.0,0.0,0.0,0.0);
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0,2.0,0.2,8.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0.2,-1.0,1.5,0.0,0.2,0.2,0.0,0.0,1.0);
  lighting();

  glShadeModel(GL_SMOOTH);
  glutDisplayFunc(&display_main);
  glutKeyboardFunc(&keyboard);
}

void camera_window(){
  glutInitWindowSize(img_width,img_height);
  glutInitWindowPosition(500,100);
  sensorwindow = glutCreateWindow("Camera Process");
  glClearColor(0.0f,0.0f,1.0f,1.0f);
  glutDisplayFunc(&display_sensor);
  glutKeyboardFunc(&keyboard);
}

void camera_backtopwindow(){
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(500,300);
  glutInitWindowPosition(500,400);
  backtopwindow = glutCreateWindow("Camera Back Top");
  glClearColor(0.0,0.0,0.0,0.0);
  glutDisplayFunc(&display_backtop);
  glutKeyboardFunc(&keyboard);
}

void ir_window(){
  glutInitWindowSize(img_width,img_height);
  glutInitWindowPosition(500,200);
  irwindow = glutCreateWindow("Tampak Sensor");
  glClearColor(0.0f,0.0f,0.0f,1.0f);
  glutDisplayFunc(&display_ir);
  glutKeyboardFunc(&keyboard);
}

void display_sensor(){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0f,0.0f,0.0f,0.0f);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float floor_x = 0.145+0.005, floor_y = 0,floor_z = 0;
  float sense_x = 0.145      , sense_y = 0,sense_z = 0.2;

  float floor_x_ = newx(floor_x, floor_y);
  float floor_y_ = newy(floor_x, floor_y);
  float sense_x_ = newx(sense_x, sense_y);
  float sense_y_ = newy(sense_y, sense_y);

  glFrustum(-0.05,0.05,0.01,-0.01,0.2,1);
  gluLookAt(sense_x_,sense_y_,sense_z,floor_x_,floor_y_,floor_z,0.0,0.0,1.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  disp_floor(0);
  // disp_robot();
  lighting();
  glShadeModel(GL_SMOOTH);
  glutSwapBuffers();

  // set luminance value to be 1 max
  glPixelTransferf(GL_RED_SCALE,0.3333*0.2);
  glPixelTransferf(GL_GREEN_SCALE,0.3334*0.2);
  glPixelTransferf(GL_BLUE_SCALE,0.3333*0.2);

  glReadPixels(0,0,img_width,img_height,GL_LUMINANCE,GL_UNSIGNED_BYTE,image_raw);

  ir8 = (image_raw[50][sen8] < 50) ? 1:0;
  ir7 = (image_raw[50][sen7] < 50) ? 1:0;
  ir6 = (image_raw[50][sen6] < 50) ? 1:0;
  ir5 = (image_raw[50][sen5] < 50) ? 1:0;
  ir4 = (image_raw[50][sen4] < 50) ? 1:0;
  ir3 = (image_raw[50][sen3] < 50) ? 1:0;
  ir2 = (image_raw[50][sen2] < 50) ? 1:0;
  ir1 = (image_raw[50][sen1] < 50) ? 1:0;

  image_raw[50][sen8] = ir8*255;
  image_raw[50][sen7] = ir7*255;
  image_raw[50][sen6] = ir6*255;
  image_raw[50][sen5] = ir5*255;
  image_raw[50][sen4] = ir4*255;
  image_raw[50][sen3] = ir3*255;
  image_raw[50][sen2] = ir2*255;
  image_raw[50][sen1] = ir1*255;
}

void display_ir(){
  glClear(GL_COLOR_BUFFER_BIT);
  glDrawPixels(img_width, img_height, GL_LUMINANCE, GL_UNSIGNED_BYTE, image_raw);
  glutSwapBuffers();
}

void display_backtop(){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.0,0.0,0.0,0.0);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0,1.25,0.01,2);
  float sense_x=-0.2,sense_y=0,sense_z=0.3;
  float floor_x=0.3,floor_y=0.0, floor_z=0.0;
  float floor_x_ = newx(floor_x, floor_y);
  float floor_y_ = newy(floor_x, floor_y);
  float sense_x_ = newx(sense_x, sense_y);
  float sense_y_ = newy(sense_y, sense_y);
  gluLookAt(sense_x_,sense_y_,sense_z,floor_x_,floor_y_,floor_z,0.0,0.0,1.0);
  glMatrixMode(GL_MODELVIEW);
  lighting();
  disp_floor(0);
  disp_robot();
  glutSwapBuffers();
}

void display_main(){
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  disp_floor(1);
  disp_robot();
  glutSwapBuffers();
}

int loadGLTexture(const char *filename, int width, int height){
  free(data);
  fileimage = fopen(filename,"r");
  if(fileimage == NULL) return 0;
  data = (unsigned char*) malloc(width*height*3);

  data_file = fread(data, width * height * 3, 1, fileimage);
  fclose(fileimage);

  unsigned int textureID;
  int border = 0;
  int depth = width*height*3;
  glGenTextures(1, &textureID);

  glBindTexture(GL_TEXTURE_2D,textureID);
  glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,width,height,0,GL_RGB, GL_UNSIGNED_BYTE,data);
  return textureID;
}

void arrowKeyPress(int key, int x, int y){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // float mv_jalan = 0.05;
  // float mv_putar = 90;
	// if(manual == 1){
  //   switch(key){
  //     case GLUT_KEY_RIGHT:
  //       mv_putar = 90;
  //       putar_robot += mv_putar;
  //       break;
  //     case GLUT_KEY_LEFT:
  //       mv_putar = -90;
  //       putar_robot += mv_putar;
  //       break;
  //     case GLUT_KEY_UP:
  //       jalan_robot[1] += mv_jalan*cos(putar_robot*DTR);
  //       jalan_robot[2] += mv_jalan*sin(putar_robot*DTR);
  //       break;
  //     case GLUT_KEY_DOWN:
  //       jalan_robot[1] -= mv_jalan*cos(putar_robot*DTR);
  //       jalan_robot[2] -= mv_jalan*sin(putar_robot*DTR);
  //       break;
  //   }
  // }
  glutPostRedisplay();		// redraw the image now
}

void keyboard(unsigned char key, int i, int j){
	 switch(key){
      case ESCkey: exit(1); break;
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
      case '1': q1+=10*DTR; dq1=10*DTR; break;
      case '2': q2+=10*DTR; dq2=10*DTR; break;
      case '!': q1-=10*DTR; dq1=-10*DTR; break;
      case '@': q2-=10*DTR; dq2=-10*DTR; break;
      case '3': q1+=10*DTR; dq1=10*DTR;q2+=10*DTR;dq2=10*DTR;break;
   }
}

void drawOneLine(double x1, double y1, double x2, double y2){
  glBegin(GL_LINES);
  glVertex3f((x1),(y1),0.0);
  glVertex3f((x2),(y2),0.0); 
  glEnd();
}
   
void model_cylinder(GLUquadricObj * object, GLdouble lowerRadius,
  GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2){
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    // gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    // gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void model_box(GLfloat width, GLfloat depth, GLfloat height, GLfloat *color1, GLfloat *color2, GLfloat *color3, int color){
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
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    glVertex3f(-width,-depth,-height);
    glVertex3f( width,-depth,-height);
    glVertex3f( width, depth,-height);
    glVertex3f(-width, depth,-height);
   glEnd();
   glBegin(GL_QUAD_STRIP);
// sides
    if (color==1) 
	    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color3);
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

void disp_floor(int grid){
  int i,j,flagc=1;
  
  if(grid){
    glPushMatrix();
    GLfloat dx=4.5,dy=4.5;
    GLint amount=15;
    GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount;
    GLfloat y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

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
    glBegin(GL_POLYGON);
    // urutan koordinat
    glTexCoord2f(0,1);glVertex3f(-1.0,-1.0,0.0);
    glTexCoord2f(0,0);glVertex3f(-1.0,1.0,0.0);
    glTexCoord2f(1,0);glVertex3f(1.0,1.0,0.0);
    glTexCoord2f(1,1);glVertex3f(1.0,-1.0,0.0);
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
  glPopMatrix();
}

void lighting(void){

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

void disp_robot(){
  glPushMatrix();
    glTranslatef(rx,ry,0.02);
    glRotatef(shi*RTD,0,0,1);
    model_box(0.05,(0.09-0.005)*2,0.01,gray8, gray7, gray6,1);
    glPushMatrix();
      glTranslatef(0,0.09,0);
      glRotatef(-90,1,0,0);
      model_cylinder(obj, 0.025/2, 0.025/2, 0.01, 2,blue3,blue3);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(0,-0.09,0);
      glRotatef(90,1,0,0);
      model_cylinder(obj, 0.025/2, 0.025/2, 0.01, 2, blue3,blue3);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(0.075,0,0);
      model_box(0.15,0.01,0.01,gray8, gray7, gray6,1);
    glPopMatrix();

    glPushMatrix();
      glTranslatef(0.15,0,0);
      model_box(0.03,0.1,0.01,red,red,red,1);
    glPopMatrix();
  glPopMatrix();
}

void jacobian(float &dx, float &dy, float &dshi, float dq2, float dq1, float shi){
  dx = 0.025/2.0*cos(shi)*(dq2 + dq1);
  dy = 0.025/2.0*sin(shi)*(dq2 + dq1);
  dshi = 0.025/0.18*(dq2-dq1);
}

void animate(int k){
  static int oldsensor = 0;
  int sensor = ((ir1)?-4:0)+((ir2)?-3:0)+((ir3)?-2:0)+((ir4)?-1:0)+((ir5)?1:0)+((ir6)?2:0)+((ir7)?3:0)+((ir8)?4:0);
  int adasensor = ir1+ir2+ir3+ir4+ir5+ir6+ir7+ir8;
  if(adasensor){
    if(sensor > 0){q1+=Kp*DTR;dq1=Kp*DTR;}
    if(sensor < 0){q2+=Kp*DTR;dq2=Kp*DTR;}
    if(sensor==0){q1+=Kp*DTR;dq1=Kp*DTR;q2+=Kp*DTR;dq2=Kp*DTR;}

    if(sensor==0||(abs(sensor+oldsensor)<2)){q1+=Kp*DTR;dq1=Kp*DTR;q2+=Kp*DTR;dq2=Kp*DTR;}
    // else{dq1=(float)sensor/30.0*Kp*DTR;q1+=dq1;dq2=-(float)sensor/30.0*Kp*DTR;q2+=dq2;}
  }
  jacobian(dx,dy,dshi,dq2,dq1,shi);
  dq2=0;dq1=0;
  shi=0.025/0.18*(q2-q1)+0;
  rv=dx*cos(shi)+dy*sin(shi);
  rx=rx+rv*cos((shi+shi_old)/2.0);
  ry=ry+rv*sin((shi+shi_old)/2.0);
  shi_old = shi;
  usleep(100000);
}

void simulation(void){
  static int count = 0;
  glutSetWindow(window);
  animate(count);
  display_main();
  glutSetWindow(backtopwindow);
  display_backtop();
  glutSetWindow(sensorwindow);
  display_sensor();
  glutSetWindow(irwindow);
  display_ir();
  usleep(2000);
  // glutPostRedisplay();
}

// void init(void){ 
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
//    lighting();
//    glShadeModel(GL_SMOOTH); 
//    glutDisplayFunc (&display) ;
//    glutKeyboardFunc(&keyboard);
//    glutSpecialFunc(&arrowKeyPress);
// }