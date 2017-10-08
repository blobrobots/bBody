# add link directories (-L)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# define dependencies path
set(BLOB_LIBS_DIR ${PROJECT_SOURCE_DIR}/../../libs)
set(BLOB_MATH_DIR ${BLOB_LIBS_DIR}/math)

# add include directories (-I)
include_directories(${BLOB_MATH_DIR}/include)

# build binaries
if(${PLATFORM} MATCHES "Arduino")
  set(ARDUINO_DEFAULT_BOARD leonardo) # Default Board ID, when not specified
  # serial port if externally defined
  if(${PORT} MATCHES "/dev/ttyACM")
    set(ARDUINO_PORT ${PORT})
    message("-- Configuring ${PROJECT_NAME} to be uploaded to port ${ARDUINO_PORT}")
  else(${PORT} MATCHES "/dev/ttyACM")
    set(ARDUINO_PORT /dev/ttyACM0)
    message("-- Configuring ${PROJECT_NAME} to be uploaded to default port ${ARDUINO_PORT}")
  endif(${PORT} MATCHES "/dev/ttyACM")
  generate_arduino_firmware(test_pwm
      SRCS test_arduino_pwm.cpp
      LIBS blob_pwm 
      PORT ${ARDUINO_PORT}
      BOARD leonardo      
      AFLAGS -cavr911)
else(${PLATFORM} MATCHES "Arduino")
#  add_executable(test_pwm_lnx test_linux.cpp) # build executable
#  target_link_libraries(test_pwm_lnx blob_pwm) # link libraries
endif(${PLATFORM} MATCHES "Arduino")
