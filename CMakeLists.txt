cmake_minimum_required(VERSION 2.7)
project(tracked_motion)

# to find SDL2
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${tracked_motion_SOURCE_DIR}/cmake")

add_definitions(-DDRAWSTUFF_TEXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/textures")
add_definitions(-DPOINTCLOUDS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/pointclouds")

find_package(OpenGL REQUIRED )
find_package(GLUT REQUIRED )

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree sample_consensus)
find_package(ompl REQUIRED)
find_package(SDL2 REQUIRED)

# this code uses the drawstuff library which is normally private to ODE, so we need to hack it from the local build
set(ODE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../ode/include)
file(GLOB ODE_LIBRARIES "${CMAKE_SOURCE_DIR}/../ode/ode/src/.libs/*.so")
file(GLOB DRAWSTUFF_LIBRARIES "${CMAKE_SOURCE_DIR}/../ode/drawstuff/src/.libs/*.a")

set(FLAGS "-DLINUX -DAMD64")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAGS}")
set(CTMAKE_CXX_FTLAGS "${CMAKE_CXX_FLAGS} ${FLAGS}")

set(LIBS
        ${OpenGL_LIBRARIES} GL GLU # the last two are not loaded for some reason
        ${GLUT_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${SDL2_LIBRARY}
        ${OMPL_LIBRARIES}
        ${ODE_LIBRARIES}
        ${DRAWSTUFF_LIBRARIES}
        X11 Xxf86vm Xrandr pthread Xi
)

include_directories(
        include
        ${OpenGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${ODE_INCLUDE_DIRS}
)

set(BASE_SOURCES
        src/Track.cpp
        src/TrackKinematicModel.cpp
        src/TrackedVehicle.cpp
        src/SkidSteeringVehicle.cpp
        src/Environment.cpp
        src/TrackedVehicleEnvironment.cpp
        src/SkidSteerVehicleEnvironment.cpp
        src/ODEUtils.cpp
        src/utils.cpp
        src/SimpleTrackedVehicle.cpp
        src/SimpleTrackedVehicleEnvironment.cpp)

add_executable(simulator ${BASE_SOURCES} src/simulator.cpp)
target_link_libraries(simulator ${LIBS})

add_executable(planner ${BASE_SOURCES}  src/OMPLTVSControlSpace.cpp src/OMPLTVSEnvironment.cpp src/OMPLTVSSimpleSetup.cpp src/OMPLTVSStatePropagator.cpp src/OMPLTVSStateSpace.cpp src/OMPLTVSStateValidityChecker.cpp src/planner.cpp)
target_link_libraries(planner ${LIBS})

add_executable(search-vis ${BASE_SOURCES} src/search_vis.cpp)
target_link_libraries(search-vis ${LIBS})