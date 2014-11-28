CFLAGS := -ggdb -DdDOUBLE -I/opt/local/include
CXXFLAGS := $(CFLAGS)
LDLIBS := -L/opt/local/lib -lstdc++ -lm -lGL -lGLU -lglut -lode -lX11 -lXxf86vm -lXrandr -lpthread -lXi

.PHONY: clean

main: main.o

clean:
	rm -f *.o main
