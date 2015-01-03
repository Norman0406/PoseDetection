cmake_minimum_required(VERSION 2.8)
project(pose)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)

# find boost
find_package(Boost 1.56 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# find eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# find opencv
set(OpenCV_STATIC ON)
find_package(OpenCV 2.4.9 REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})

#message(${OpenCV_LIB_DIR})
#message(${OpenCV_LIBS})
#message(${_OpenCV_LIB_PATH})

set(SOURCEFILES
        src/input/depthcamera.cpp
        src/segmentation/connectedcomponentlabeling.cpp
        src/segmentation/staticmap.cpp
        src/segmentation/tracking.cpp
        src/tracking/bone.cpp
        src/tracking/fitting.cpp
        src/tracking/joint.cpp
        src/tracking/skeleton.cpp
        src/tracking/skeletonupperbody.cpp
        src/utils/boundingbox2d.cpp
        src/utils/boundingbox3d.cpp
        src/utils/connectedcomponent.cpp
        src/utils/exception.cpp
        src/utils/module.cpp
        src/utils/numberedfilereader.cpp
        src/utils/numberedfilewriter.cpp
        src/utils/timer.cpp
        src/utils/utils.cpp
)

set(HEADERFILES
        include/pose.h
        src/input/depthcamera.h
        src/segmentation/connectedcomponentlabeling.h
        src/segmentation/staticmap.h
        src/segmentation/tracking.h
        src/tracking/bone.h
        src/tracking/fitting.h
        src/tracking/joint.h
        src/tracking/skeleton.h
        src/tracking/skeletonupperbody.h
        src/utils/boundingbox2d.h
        src/utils/boundingbox3d.h
        src/utils/connectedcomponent.h
        src/utils/exception.h
        src/utils/module.
        src/utils/numberedfilereader.h
        src/utils/numberedfilewriter.h
        src/utils/timer.h
        src/utils/utils.h
)

add_library(pose ${SOURCEFILES})
target_link_libraries(pose ${OPENCV_LIBS} ${Boost_LIBRARIES})
