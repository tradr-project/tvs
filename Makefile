CFLAGS := -ggdb -DdDOUBLE
LDLIBS := -lstdc++ -lm -lode

DRAWSTUFF_DEPS = drawstuff.o

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
        CCFLAGS += -DLINUX
        LDLIBS += -lGL -lGLU -lglut -lX11 -lXxf86vm -lXrandr -lpthread -lXi
        DRAWSTUFF_DEPS += drawstuff_x11.o
    endif
    ifeq ($(UNAME_S),Darwin)
        CFLAGS += -DOSX -I/opt/local/include
        LDLIBS += -L/opt/local/lib -framework Cocoa -framework CoreVideo -framework IOKit -framework OpenGL -framework GLUT
        DRAWSTUFF_DEPS += drawstuff_osx.o
    endif
    UNAME_P := $(shell uname -p)
    ifeq ($(UNAME_P),x86_64)
        CCFLAGS += -DAMD64
    endif
    ifneq ($(filter %86,$(UNAME_P)),)
        CCFLAGS += -DIA32
    endif
#    ifneq ($(filter arm%,$(UNAME_P)),)
#        CCFLAGS += -DARM
#    endif
endif

CXXFLAGS := $(CFLAGS)

.PHONY: clean

main: main.o $(DRAWSTUFF_DEPS)

clean:
	rm -f *.o main
