PCL_MODULES=common filters io kdtree sample_consensus
PCL_VERSION=1.7
PCL_CFLAGS = $(shell sh -c 'for i in $(PCL_MODULES); do pkg-config pcl_$${i}-$(PCL_VERSION) --cflags; done')
PCL_LDLIBS = $(shell sh -c 'for i in $(PCL_MODULES); do pkg-config pcl_$${i}-$(PCL_VERSION) --libs; done')

SDL_CFLAGS = $(shell pkg-config sdl2 --cflags)
SDL_LDLIBS = $(shell pkg-config sdl2 --libs)

ODE_CFLAGS = -DDRAWSTUFF_TEXTURE_PATH=\"$(PWD)/textures\" -DPOINTCLOUDS_PATH=\"$(PWD)/pointclouds\" -I$(PWD)/../ode/include
ODE_LDLIBS = -L$(PWD)/../ode/ode/src/.libs -lode -L$(PWD)/../ode/drawstuff/src/.libs -ldrawstuff

OMPL_CFLAGS = -I$(PWD)/../ompl/src
OMPL_LDLIBS = -L$(PWD)/../ompl/build/Release/lib -lompl

CFLAGS := -std=c++03 -O0 -ggdb $(ODE_CFLAGS) $(OMPL_CFLAGS) $(SDL_CFLAGS) $(OPT_CFLAGS) -DHAVE_JOYSTICK
LDLIBS := -lm -lstdc++ $(ODE_LDLIBS) $(OMPL_LDLIBS) $(SDL_LDLIBS)

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
        LDLIBS += -lGL -lGLU -lglut -lX11 -lXxf86vm -lXrandr -lpthread -lXi -lboost_system -lboost_thread -lboost_filesystem
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
        LDLIBS += -framework Cocoa -framework CoreVideo -framework IOKit -framework OpenGL -framework GLUT -lboost_system-mt -lboost_thread-mt -lboost_filesystem-mt
    endif
endif

CXXFLAGS = $(CFLAGS)

.PHONY: clean all

# Heightfield.o TriMesh.o PointCloud.o
TVS_OBJS := Track.o TrackKinematicModel.o TrackedVehicle.o SkidSteeringVehicle.o Environment.o ODEUtils.o simulator.o search_vis.o utils.o
OMPL_OBJS := OMPLTVSControlSpace.o OMPLTVSEnvironment.o OMPLTVSSimpleSetup.o OMPLTVSStatePropagator.o OMPLTVSStateSpace.o OMPLTVSStateValidityChecker.o planner.o
OBJS = $(TVS_OBJS) $(OMPL_OBJS)


all: simulator planner search_vis

simulator: Track.o TrackKinematicModel.o TrackedVehicle.o SkidSteeringVehicle.o Environment.o ODEUtils.o utils.o simulator.o
	$(CC) $^ $(LDLIBS) -o $@

planner: Track.o TrackKinematicModel.o TrackedVehicle.o SkidSteeringVehicle.o Environment.o ODEUtils.o OMPLTVSControlSpace.o OMPLTVSEnvironment.o OMPLTVSSimpleSetup.o OMPLTVSStatePropagator.o OMPLTVSStateSpace.o OMPLTVSStateValidityChecker.o utils.o planner.o
	$(CC) $^ $(LDLIBS) -o $@

search_vis: Track.o TrackKinematicModel.o TrackedVehicle.o SkidSteeringVehicle.o Environment.o ODEUtils.o utils.o search_vis.o
	$(CC) $^ $(LDLIBS) -o $@

-include $(OBJS:.o=.d)

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $*.cpp -o $*.o
	$(CXX) -MM $(CXXFLAGS) $*.cpp > $*.d
	@cp -f $*.d $*.d.tmp
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

clean:
	@rm -vf $(OBJS) $(OBJS:.o=.d) simulator planner search_vis
	@sh -c 'if ls *.o > /dev/null 2>&1; then echo "error: some *.o files have not been cleaned: $$(ls *.o)"; false; fi'

