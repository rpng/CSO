CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(calibration_stereo_odom)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set (CSO_SRC_DIR src)
set (CSO_HEAD_DIR include)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# C++11 support
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
FIND_PACKAGE(Cholmod)
FIND_PACKAGE(BLAS)
FIND_PACKAGE(LAPACK)
FIND_PACKAGE(CSparse)
FIND_PACKAGE(G2O)

include_directories(
  ${CSO_HEAD_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/ThirdParty/libviso2/src
)

FILE(GLOB CSO_HEAD_FILES "include/*.h")
FILE(GLOB CSO_SRC_FILES "src/*.cpp")

IF(BUILD_LIB)
   add_library(calibration_stereo_odom ${CSO_HEAD_FILES} ${CSO_SRC_FILES})
ELSE()
   add_executable(calibration_stereo_odom ${CSO_HEAD_FILES} ${CSO_SRC_FILES})
ENDIF()

target_link_libraries (calibration_stereo_odom 
  ${EIGEN3_LIBS}
  ${G2O_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/ThirdParty/libviso2/build/libviso2.so
  cholmod
  png
)
