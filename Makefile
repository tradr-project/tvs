CFLAGS := -ggdb -DDRAWSTUFF_TEXTURE_PATH=\"$(PWD)/textures\" -I$(PWD)/../ode/include
LDLIBS := -lstdc++ -lm -L$(PWD)/../ode/ode/src/.libs -lode -L$(PWD)/../ode/drawstuff/src/.libs -ldrawstuff

ifeq ($(OS),Windows_NT)
#    CCFLAGS += -DWIN32
#    ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
#        CCFLAGS += -DAMD64
#    endif
#    ifeq ($(PROCESSOR_ARCHITECTURE),x86)
#        CCFLAGS += -DIA32
#    endif
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        CFLAGS += -DLINUX
        LDLIBS += -lGL -lGLU -lglut -lX11 -lXxf86vm -lXrandr -lpthread -lXi
    endif
    ifeq ($(UNAME_S),Darwin)
        CFLAGS += -DOSX
        LDLIBS += -framework Cocoa -framework CoreVideo -framework IOKit -framework OpenGL -framework GLUT
    endif
    UNAME_P := $(shell uname -p)
    ifeq ($(UNAME_P),x86_64)
        CFLAGS += -DAMD64
    endif
    ifneq ($(filter %86,$(UNAME_P)),)
        CFLAGS += -DIA32
    endif
#    ifneq ($(filter arm%,$(UNAME_P)),)
#        CCFLAGS += -DARM
#    endif
endif

.PHONY: clean

main: main.o

clean:
	rm -f *.o main
