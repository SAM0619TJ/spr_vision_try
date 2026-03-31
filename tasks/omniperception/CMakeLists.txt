cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED OpenVINO_DIR)
  if(APPLE)
    find_path(_OV_CMAKE_DIR "OpenVINOConfig.cmake"
      PATHS
        "$ENV{HOME}/intel/openvino/runtime/cmake"
        "/usr/local/lib/cmake/openvino"
        "/opt/homebrew/lib/cmake/openvino"
      NO_DEFAULT_PATH
    )
    if(_OV_CMAKE_DIR)
      set(OpenVINO_DIR "${_OV_CMAKE_DIR}")
    endif()
  else()
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
  endif()
endif()

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(omniperception OBJECT
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)