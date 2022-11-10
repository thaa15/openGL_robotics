# for Opengl
 
CCGL=g++
GL_CFLAGS = -O2 -DSHM -DHZ=100
CYG_OpenGL = -lglut -lGLU -lGL
GL_LIBS += $(CYG_OpenGL) -lm 

exercise: planargl.c 
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o planargl.o
	./planargl.o

clean:
	rm -rf *.o