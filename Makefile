CFLAGS := -ggdb -DDRAWSTUFF_TEXTURE_PATH=\"$(PWD)/textures\" -DPOINTCLOUDS_PATH=\"$(PWD)/pointclouds\" -I$(PWD)/../ode/include -I/opt/local/include
LDLIBS := -lm -L$(PWD)/../ode/ode/src/.libs -lode -L$(PWD)/../ode/drawstuff/src/.libs -ldrawstuff -lstdc++ -L/opt/local/lib -lboost_system-mt -lompl

ifeq ($(OS),Windows_NT)
    CFLAGS += -DWIN32
    ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
        CFLAGS += -DAMD64
    endif
    ifeq ($(PROCESSOR_ARCHITECTURE),x86)
        CFLAGS += -DIA32
    endif
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        CFLAGS += -DLINUX
		UNAME_P := $(shell uname -p)
		ifeq ($(UNAME_P),x86_64)
			CFLAGS += -DAMD64
		endif
		ifneq ($(filter %86,$(UNAME_P)),)
			CFLAGS += -DIA32
		endif
		ifneq ($(filter arm%,$(UNAME_P)),)
			CFLAGS += -DARM
		endif
        LDLIBS += -lGL -lGLU -lglut -lX11 -lXxf86vm -lXrandr -lpthread -lXi
    endif
    ifeq ($(UNAME_S),Darwin)
        CFLAGS += -DOSX
		UNAME_M := $(shell uname -m)
		ifeq ($(UNAME_M),x86_64)
			CFLAGS += -DAMD64
		endif
		ifneq ($(filter %86,$(UNAME_M)),)
			CFLAGS += -DIA32
		endif
        LDLIBS += -framework Cocoa -framework CoreVideo -framework IOKit -framework OpenGL -framework GLUT
    endif
endif

CXXFLAGS = $(CFLAGS)

.PHONY: clean all

OBJS := Heightfield.o PointCloud.o Track.o TrackKinematicModel.o TrackedVehicle.o Environment.o Planner.o ODEUtils.o OMPLEnvironment.o OMPLGoalRegion.o OMPLStateProjectionEvaluator.o OMPLStateSpace.o OMPLStatePropagator.o main.o
TARGET = main

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDLIBS) -o $(TARGET)

-include $(OBJS:.o=.d)

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $*.cpp -o $*.o
	$(CXX) -MM $(CXXFLAGS) $*.cpp > $*.d
	@cp -f $*.d $*.d.tmp
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

clean:
	rm -f $(OBJS) $(OBJS:.o=.d) $(TARGET)
