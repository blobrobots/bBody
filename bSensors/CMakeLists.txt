if(${PLATFORM} MATCHES "Arduino")
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_ROOT}/Modules/ArduinoToolchain.cmake) # Arduino Toolchain
  set(ARDUINO_AVRDUDE_PROGRAM avrdude.py) # override arvdude for upload to micro and leonardo
endif(${PLATFORM} MATCHES "Arduino")

cmake_minimum_required (VERSION 2.8.5)
cmake_policy(VERSION 2.8.5)
project(bSensors)

# flags for debug version: "-DCMAKE_BUILD_TYPE=Debug/Release"
if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  message("-- Configuring ${PROJECT_NAME} for Debug")
  add_definitions(-D__DEBUG__)
  set(CMAKE_CC_FLAGS "${CMAKE_CC_CFLAGS} -Wall -Werror -g -O0 -fPIC")
else(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set(CMAKE_CC_FLAGS "${CMAKE_CC_CFLAGS} -Wall -Werror -O2 -fPIC")
endif(${CMAKE_BUILD_TYPE} MATCHES "Debug")

# define dependencies path
set(BLOB_LIBS_DIR ${PROJECT_SOURCE_DIR}/../../bLibs)
set(BLOB_6DOF_DIR ${BLOB_LIBS_DIR}/b6dof)
set(BLOB_RT_DIR ${BLOB_LIBS_DIR}/bRt)
set(BLOB_TYPE_DIR ${BLOB_LIBS_DIR}/bTypes)
set(BLOB_SIGNAL_DIR ${PROJECT_SOURCE_DIR}/../bSignal)

# add include directories (-I)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${BLOB_6DOF_DIR}/include)
include_directories(${BLOB_TYPE_DIR}/include)
include_directories(${BLOB_RT_DIR}/include)
include_directories(${BLOB_SIGNAL_DIR}/include)

# link include directories (-L)
link_directories(${BLOB_RT_DIR}/lib)
link_directories(${BLOB_SIGNAL_DIR}/lib)

# libs to link
set(EXTRA_LIBS blob_i2c blob_rt)
set(EXTRA_LIBS_static blob_i2c_static blob_rt_static)

# sources
set(MPU6050_SRC src/bMpu6050.cpp)
set(HMC5883_SRC src/bHmc5883.cpp)
set(MS5611_SRC src/bMs5611.cpp)

# output files path: libs at /lib and executables at bin/
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# build the shared and static libraries
if(${PLATFORM} MATCHES "Arduino")
  set(ARDUINO_DEFAULT_BOARD leonardo) # Default Board ID, when not specified
  set(ARDUINO_DEFAULT_PORT /dev/ttyACM0) # Default Port, when not specified
  generate_arduino_library(blob_mpu6050
    SRCS ${MPU6050_SRC}
    LIBS ${EXTRA_LIBS}
    BOARD leonardo)
  generate_arduino_library(blob_hmc5883
    SRCS ${HMC5883_SRC}
    LIBS ${EXTRA_LIBS}
    BOARD leonardo)
  generate_arduino_library(blob_ms5611
    SRCS ${MS5611_SRC}
    LIBS ${EXTRA_LIBS}
    BOARD leonardo)
else(${PLATFORM} MATCHES "Arduino")
  add_library(blob_mpu6050 SHARED ${MPU6050_SRC})
  target_link_libraries(blob_mpu6050 ${EXTRA_LIBS})
  add_library(blob_mpu6050_static STATIC ${MPU6050_SRC})
  target_link_libraries(blob_mpu6050_static ${EXTRA_LIBS_STATIC})

  add_library(blob_hmc5883 SHARED ${HMC5883_SRC})
  target_link_libraries(blob_hmc5883 ${EXTRA_LIBS})
  add_library(blob_hmc5883_static STATIC ${HMC5883_SRC})
  target_link_libraries(blob_hmc5883_static ${EXTRA_LIBS_STATIC})

  add_library(blob_ms5611 SHARED ${MS5611_SRC})
  target_link_libraries(blob_ms5611 ${EXTRA_LIBS})
  add_library(blob_ms5611_static STATIC ${MS5611_SRC})
  target_link_libraries(blob_ms5611_static ${EXTRA_LIBS_STATIC})
endif(${PLATFORM} MATCHES "Arduino")

# compile tests and dependencies only if standalone compilation
string(FIND ${CMAKE_BINARY_DIR} ${PROJECT_NAME} IS_PROJECT) 
if("${IS_PROJECT}" GREATER -1)
  # compile tests
#  add_subdirectory(test) 
  # dependencies
  add_subdirectory(${BLOB_RT_DIR} "${CMAKE_CURRENT_BINARY_DIR}/rt")
  add_subdirectory(${BLOB_SIGNAL_DIR} "${CMAKE_CURRENT_BINARY_DIR}/signal")
endif("${IS_PROJECT}" GREATER -1)
