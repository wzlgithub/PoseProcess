CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#PROJECT()
#ADD_SUBDIRECTORY(src)
ADD_DEFINITIONS(-std=c++11)
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES("/usr/include/eigen3")
#INCLUDE_DIRECTORIES("/home/wzl/cprogram/JZ_calibrate/project/PoseProcess/src")
#LINK_DIRECTORIES(${OpenCV_LIBS})
#ADD_LIBRARY(PoseProcess STATIC PoseProcess.cpp)
ADD_EXECUTABLE(PoseProcess main.cpp)
TARGET_LINK_LIBRARIES(PoseProcess ${OpenCV_LIBS})

