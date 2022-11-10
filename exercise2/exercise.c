#include "libExercise.c"

int main(int argc, char** argv){
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );

    obj = gluNewQuadric();
    main_window();
    textureNumber = loadGLTexture("track2.ppm",500,500);
    camera_backtopwindow();
    textureNumber = loadGLTexture("track2.ppm",500,500);
    camera_window();
    textureNumber = loadGLTexture("track2.ppm",500,500);
    ir_window();

    glutIdleFunc(&simulation);
    glutMainLoop();
    return 0;
}

// Sensor terbaca 
// nilai sensor bener 
// controller 
// kp yang paling tepat 
// advanced controller 