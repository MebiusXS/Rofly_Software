execute_process(COMMAND "/home/fast/rocon_ws/build/rocon_devices/rocon_hue/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/fast/rocon_ws/build/rocon_devices/rocon_hue/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
