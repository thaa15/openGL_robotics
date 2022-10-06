#include "libExercise.c"

int main(int argc, char** argv){
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
    glutInitWindowSize(1200,800);	
    glutInitWindowPosition (300, 100);

    window = glutCreateWindow ("Thariq Hadyan");
    init();
    glutIdleFunc(&simulation);
    glutMainLoop();
    return 0;
}