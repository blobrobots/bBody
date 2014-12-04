if(${PLATFORM} MATCHES "Arduino")
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_ROOT}/Modules/ArduinoToolchain.cmake) # Arduino Toolchain
  set(ARDUINO_AVRDUDE_PROGRAM avrdude.py) # override arvdude for upload to micro and leonardo
endif(${PLATFORM} MATCHES "Arduino")

cmake_minimum_required (VERSION 2.8.5)
cmake_policy(VERSION 2.8.5)
project(bat)

# flags for debug version: "-DCMAKE_BUILD_TYPE=Debug/Release"
if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  message("-- Configuring ${PROJECT_NAME} for Debug")
  add_definitions(-D__DEBUG__)
  set(CMAKE_CC_FLAGS "${CMAKE_CC_CFLAGS} -Wall -Werror -g -O0 -fPIC")
else(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set(CMAKE_CC_FLAGS "${CMAKE_CC_CFLAGS} -Wall -Werror -O2 -fPIC")
endif(${CMAKE_BUILD_TYPE} MATCHES "Debug")

# define dependencies path
set(BLOB_LIBS_DIR ${PROJECT_SOURCE_DIR}/../../libs)
set(BLOB_6DOF_DIR ${BLOB_LIBS_DIR}/6dof)
set(BLOB_RT_DIR ${BLOB_LIBS_DIR}/rt)
set(BLOB_TYPE_DIR ${BLOB_LIBS_DIR}/types)
set(BLOB_BRAIN_DIR ${PROJECT_SOURCE_DIR}/../../brain)
set(BLOB_PERCEPTION_DIR ${BLOB_BRAIN_DIR}/perception)
set(BLOB_COMMS_DIR ${PROJECT_SOURCE_DIR}/../../comms)
set(BLOB_SIGNAL_DIR ${PROJECT_SOURCE_DIR}/../signal)
set(BLOB_SENSOR_DIR ${PROJECT_SOURCE_DIR}/../sensors)

# add include directories (-I)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${BLOB_6DOF_DIR}/include)
include_directories(${BLOB_TYPE_DIR}/include)
include_directories(${BLOB_RT_DIR}/include)
include_directories(${BLOB_PERCEPTION_DIR}/include)
include_directories(${BLOB_COMMS_DIR}/include)
include_directories(${BLOB_SIGNAL_DIR}/include)
include_directories(${BLOB_SENSOR_DIR}/include)

# link include directories (-L)
link_directories(${BLOB_RT_DIR}/lib)
link_directories(${BLOB_COMMS_DIR}/lib)
link_directories(${BLOB_PERCEPTION_DIR}/lib)
link_directories(${BLOB_SIGNAL_DIR}/lib)
link_directories(${BLOB_SENSOR_DIR}/lib)

# libs to link
set(EXTRA_LIBS blob_mpu6050 
               blob_hmc5883 
               blob_ms5611
               blob_i2c
               blob_rt
               blob_comms
               blob_perception)

set(EXTRA_LIBS_static blob_mpu6050_static
                      blob_hmc5883_static 
                      blob_ms5611_static 
                      blob_i2c_static
                      blob_rt_static
                      blob_comms_static
                      blob_perception_static)
# sources
set(BAT_SRC src/bat.cpp)

# output files path: libs at /lib and executables at bin/
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# build the shared and static libraries
if(${PLATFORM} MATCHES "Arduino")
  set(ARDUINO_DEFAULT_BOARD leonardo) # Default Board ID, when not specified
  set(ARDUINO_DEFAULT_PORT /dev/ttyACM0) # Default Port, when not specified
  generate_arduino_firmware(bBat
      SRCS ${BAT_SRC}
      LIBS ${EXTRA_LIBS} 
      PORT /dev/ttyACM0
      BOARD leonardo
      AFLAGS -cavr911)
else(${PLATFORM} MATCHES "Arduino")
#  add_executable(bBat ${BAT_SRC}) # build executable
#  target_link_libraries(bBat ${EXTRA_LIBS}) # link libraries
#  add_executable(bBat_standalone ${BAT_SRC}) # build executable
#  target_link_libraries(bBat_standalone ${EXTRA_LIBS_static}) # link libraries
endif(${PLATFORM} MATCHES "Arduino")

# compile tests and dependencies only if standalone compilation
string(FIND ${CMAKE_BINARY_DIR} ${PROJECT_NAME} IS_PROJECT) 
if("${IS_PROJECT}" GREATER -1)
  # dependencies
  add_subdirectory(${BLOB_RT_DIR} "${CMAKE_CURRENT_BINARY_DIR}/rt")
  add_subdirectory(${BLOB_COMMS_DIR} "${CMAKE_CURRENT_BINARY_DIR}/comms")
  add_subdirectory(${BLOB_PERCEPTION_DIR} "${CMAKE_CURRENT_BINARY_DIR}/perception")
  add_subdirectory(${BLOB_SIGNAL_DIR} "${CMAKE_CURRENT_BINARY_DIR}/signal")
  add_subdirectory(${BLOB_SENSOR_DIR} "${CMAKE_CURRENT_BINARY_DIR}/sensor")
endif("${IS_PROJECT}" GREATER -1)
