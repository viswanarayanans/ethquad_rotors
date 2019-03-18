execute_process(COMMAND "/home/quad/viswa_ws/Rotors_sim/build/rotors_evaluation/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/quad/viswa_ws/Rotors_sim/build/rotors_evaluation/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
