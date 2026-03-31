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

# OpenMP support (macOS needs Homebrew libomp)
find_package(OpenMP QUIET)
if(NOT OpenMP_CXX_FOUND AND APPLE)
  set(_LIBOMP_PREFIX "/opt/homebrew/opt/libomp")
  if(EXISTS "${_LIBOMP_PREFIX}/include/omp.h")
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I${_LIBOMP_PREFIX}/include")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "${_LIBOMP_PREFIX}/lib/libomp.dylib")
    find_package(OpenMP COMPONENTS CXX REQUIRED)
  endif()
endif()

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT
    armor.cpp
    aux_solver.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)
if(OpenMP_CXX_FOUND)
  target_link_libraries(auto_aim OpenMP::OpenMP_CXX)
endif()