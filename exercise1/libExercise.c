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
#include <math.h>
#define PI		3.14159265358
#define DTR     PI/180
#define RTD     180/PI
#define ESCkey	    27
#define Xoffset	    0.0	
#define Yoffset	    0.0
#define Zoffset	    0.3
#define Link1       0.3
#define Link2       0.2
#define img_height  100
#define img_width   500

GLUquadricObj *obj;
// Warna
GLfloat red[4]    = {1.0,0.0,0.0,1.0};
GLfloat black[4]  = {0.0,0.0,0.0,1.0};
GLfloat white[4]  = {1.0,1.0,1.0,1.0};
GLfloat green1[4] ={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  ={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  ={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  ={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4]  ={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4]={0.8, 0.8, 0.0, 1.0};
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
int window, sensorwindow, irwindow;
unsigned char image_raw[img_height+1][img_width+1];

int putar = 0,manual = 0,segitiga=0,lingkaran = 0;
int belok30 = 0;
double checker = 0;
double jalan_robot[3] = {0.0,0.0,0.0};
double putar_robot = 0.0;
double max_Pos = 2.0;
float mv_jalan = 0.05;
float mv_putar = 90;

void arrowKeyPress(int, int, int);
void init_robot(); // Inisialisasi Join
void keyboard(unsigned char, int , int); // onKeyPress be like
void display(void); // display robot dan workspace (terdapat fungsi disp) 
void drawOneLine(double, double, double, double); // Gambar satu garis dengan GL_LINES
void model_cylinder(GLUquadricObj *, GLdouble, GLdouble, GLdouble, GLint, GLfloat *, GLfloat *); // Gambar tabung
void model_box(GLfloat, GLfloat, GLfloat, GLfloat *, GLfloat *, GLfloat *, int); // Gambar Kotak
void disp_floor(void); // Visualisasi workspace
void lighting(void); // Memberikan warna
void disp_robot(double[3],double); // Menampilkan robot
void simulation(void); // Simulasi robot selama iddle
void init(void); // Mulai


void arrowKeyPress(int key, int x, int y){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // float mv_jalan = 0.05;
  // float mv_putar = 90;
	if(manual == 1){
    switch(key){
      case GLUT_KEY_RIGHT:
        mv_putar = 90;
        putar_robot += mv_putar;
        break;
      case GLUT_KEY_LEFT:
        mv_putar = -90;
        putar_robot += mv_putar;
        break;
      case GLUT_KEY_UP:
        jalan_robot[1] += mv_jalan*cos(putar_robot*DTR);
        jalan_robot[2] += mv_jalan*sin(putar_robot*DTR);
        break;
      case GLUT_KEY_DOWN:
        jalan_robot[1] -= mv_jalan*cos(putar_robot*DTR);
        jalan_robot[2] -= mv_jalan*sin(putar_robot*DTR);
        break;
    }
  }
  glutPostRedisplay();		// redraw the image now
}

void keyboard(unsigned char key, int i, int j){
	 switch(key){
      case ESCkey: exit(1); break;
      case 'w': glTranslatef(0,-0.1,0); break;
      case 's': glTranslatef(0,0.1,0); break;
      case 'a': glTranslatef(0.1,0,0); break;
      case 'd': glTranslatef(-0.1,0,0); break;
      case 'z':
        if(manual == 1){
          mv_putar -= 45;
          putar_robot += mv_putar;
        }
        break;
      case 'x': 
        if(manual == 1){
          mv_putar += 45;
          putar_robot += mv_putar;
        }
        break;
      case 'p':
          if(!putar)
            putar = 1;
          else if(putar){
            putar = 0;
            checker = 0.0;
            mv_jalan = 0.05;
            mv_putar = 90;
            putar_robot = 0.0;
            jalan_robot[1] = 0.0;
            jalan_robot[2] = 0.0;
          }
          break;
      case 'y':
          if(!segitiga){
            segitiga = 1;
            putar_robot = 90 - 30;
          }
          else if(segitiga){
            segitiga = 0;
            checker = 0.0;
            mv_jalan = 0.05;
            mv_putar = 90;
            putar_robot = 0.0;
            jalan_robot[1] = 0.0;
            jalan_robot[2] = 0.0;
          }
          break;
      case 'l':
          if(!lingkaran){
            lingkaran = 1;
            putar_robot = 0;
            mv_jalan = 0.02;
          }
          else if(lingkaran){
            lingkaran = 0;
            checker = 0.0;
            mv_jalan = 0.05;
            mv_putar = 90;
            putar_robot = 0.0;
            jalan_robot[1] = 0.0;
            jalan_robot[2] = 0.0;
          }
          break;
      case 'm':
          if(!manual && !segitiga && !putar){
            manual = 1;
            mv_putar = 0;
          }
          else if(manual){
            manual = 0;
            mv_jalan = 0.05;
            mv_putar = 90;
            putar_robot = 0.0;
            jalan_robot[1] = 0.0;
            jalan_robot[2] = 0.0;
          }
          break;
   }
}

void display(void){
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   //glLoadIdentity();  // Reset View
   disp_floor();
   disp_robot(jalan_robot,putar_robot);
   glutSwapBuffers() ; 
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

void disp_floor(void){
  int i,j,flagc=1;

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


void disp_robot(double maju_arah[3],double putar_arah){
  glPushMatrix();
    glRotatef(90,0,1,0);
    model_cylinder(obj, 0.025, 0.025, 0.02, 1, red, red);
    glTranslatef(maju_arah[0],maju_arah[1],maju_arah[2]);
    glRotatef(putar_arah,1,0,0);
    model_cylinder(obj, 0.025, 0.025, 0.02, 1, blue3, blue3);
    glTranslatef(0,0,0.1);
    glPushMatrix();
      glTranslatef(-0.02,0,0);
      glRotatef(-90,0,1,0);
      model_box(0.18, 0.05, 0.05, gray8, gray7, gray6,1);
    glPopMatrix();
    glTranslatef(0,0,0.1);
    glPushMatrix();
      model_cylinder(obj, 0.025, 0.025, 0.02, 1, blue3, blue3);
    glPopMatrix();
    glTranslatef(-0.02,0.07,-0.1); //y gerak
    model_box(0.05, 0.1, 0.05, gray8, gray7, gray6,1);
    glTranslatef(0,0.05,0);
    model_box(0.05, 0.01, 0.09, red, red, red,1);
  glPopMatrix();
}

void simulation(void){
  if(putar == 1){
    jalan_robot[1] += mv_jalan*cos(putar_robot*DTR);
    jalan_robot[2] += mv_jalan*sin(putar_robot*DTR);
    checker += mv_jalan;
    if(checker >= max_Pos){
      putar_robot += mv_putar;
      if(putar_robot > 360.0){ 
        putar_robot = 0.0;
      }
      if(jalan_robot[1] <= 0.05 && jalan_robot[2] <= 0.05) putar_robot = 0.0;
      checker = 0.0;
    }
  }else if(segitiga == 1){
    
    jalan_robot[1] += mv_jalan*cos(putar_robot*DTR);
    jalan_robot[2] += mv_jalan*sin(putar_robot*DTR);
  }else if(lingkaran == 1){
    putar_robot += 1.0;
    jalan_robot[1] += mv_jalan*cos(putar_robot*DTR);
    jalan_robot[2] += mv_jalan*sin(putar_robot*DTR);
    if(putar_robot > 360.0){ 
      putar_robot = 0.0;
    }
    if(jalan_robot[1] <= 0.05 && jalan_robot[2] <= 0.05) putar_robot = 0.0;
  }
  display();
  // usleep(1000);
  glutPostRedisplay();
}

void init(void){ 
   obj = gluNewQuadric(); 
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
   glShadeModel(GL_SMOOTH); 
   glutDisplayFunc (&display) ;
   glutKeyboardFunc(&keyboard);
   glutSpecialFunc(&arrowKeyPress);
}