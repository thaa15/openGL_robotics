# for Opengl

CCGL=gcc
GL_CFLAGS = -O2 
XLIBS = -lXext -lXmu -lXi -lX11
GL_LIBS =  
#GL_LIBS =  -I/usr/X11R6/include -L/usr/lib -L/usr/X11R6/lib  #gak jalan di opengl 2.x 
CYG_OpenGL = -I/usr/include/ -lglut32 -lglu32 -lopengl32 #-lcygipc
LINUX_OpenGL =  -lglut -lGLU -lGL $(XLIBS) 
GL_LIBS += $(CYG_OpenGL) -lm 

INC_FILES= $(wildcard *.cpp) $(wildcard *.o)

all:	fog lightmaterial lightposition projection shapes texture transformation

fog: fog.c glm.c
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 

projection: projection.c glm.c
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 

shapes: shapes.c
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 

transformation: transformation.c glm.c
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 

lightmaterial: lightmaterial.c glm.c
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 

lightposition: lightposition.c glm.c
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 

texture: texture.c glm.c
	$(CCGL) $(GL_CFLAGS) $^ $(GL_LIBS) -o $@ 

clean:
	rm -rf *.exe glsim