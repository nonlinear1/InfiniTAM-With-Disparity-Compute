cmake_minimum_required(VERSION 3.1)
project(InfiniTAM)

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.2")
set(CMAKE_BUILD_TYPE Release)                        #Important    make fast thread speed
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/Main/)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3.4.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)


set(SOURCES Main/InfiniTAM.cpp
            Main/InfiniTAMThread.cpp
            Main/ComputeDisparity.cpp
            Disparity/libelas/src/filter.cpp
            Disparity/libelas/src/matrix.cpp
            Disparity/libelas/src/triangle.cpp
            Disparity/libelas/src/descriptor.cpp
            Disparity/libelas/src/elas.cpp
            Disparity/BM_SGBM/blockmatching.cpp
            Disparity/BM_SGBM/sgblockmatching.cpp
            Disparity/SPS_Stereo/SGMStereo.cpp
            Disparity/SPS_Stereo/SPSStereo.cpp )

set(HEADERS Main/InfiniTAM.h
            Main/InfiniTAMThread.h
            Main/ComputeDisparity.h
            Disparity/libelas/src/filter.h
            Disparity/libelas/src/triangle.cpp
            Disparity/libelas/src/matrix.cpp
            Disparity/libelas/src/descriptor.h
            Disparity/libelas/src/elas.h
            Disparity/BM_SGBM/blockmatching.h
            Disparity/BM_SGBM/sgblockmatching.h
            Disparity/SPS_Stereo/SGMStereo.h
            Disparity/SPS_Stereo/SPSStereo.h
            Disparity/SPS_Stereo/defParameter.h )

add_subdirectory(ORUtils)
add_subdirectory(Utils)
add_subdirectory(Engine)
add_subdirectory(ITMLib)


add_executable(InfiniTAM ${HEADERS} ${SOURCES} ${EXTRA_EXECUTABLE_FLAGS})
target_link_libraries(InfiniTAM Engine Utils ORUtils)  #in ordered
target_link_libraries(InfiniTAM ${OpenCV_LIBS})
