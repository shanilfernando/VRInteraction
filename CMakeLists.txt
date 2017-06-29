cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(libfreenect2_grabber)

set(CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set(OpenCV_STATIC OFF)


find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)

get_filename_component(EIGEN_ROOT "/usr/include/eigen3" PATH)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_EIGEN eigen3)
  endif(PKG_CONFIG_FOUND)
  find_path(EIGEN_INCLUDE_DIRS Eigen/Core
    HINTS ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS} 
          "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/Eigen 3.0.0" "$ENV{PROGRAMW6432}/Eigen 3.0.0"
          "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"   
    PATH_SUFFIXES eigen3 include/eigen3 include)
  find_package_handle_standard_args(eigen DEFAULT_MSG EIGEN_INCLUDE_DIRS)
  set(EIGEN_DEFINITIONS ${EIGEN_DEFINITIONS} -DEIGEN_USE_NEW_STDVECTOR 
      -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
  "G:/SHANIL/Projects/Thesis/libfreenect2/examples/protonect/build/include/"
  ${LIBFREENECT2_THREADING_INCLUDE_DIR}
  ${LibUSB_INCLUDE_DIRS}
  ${TurboJPEG_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "G:/SHANIL/Projects/Thesis/libfreenect2/examples/protonect/src/tinythread/"
  "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/inc/"
)

file(GLOB srcs *.cpp *.h)
file(GLOB cuda Cuda/*.cu)
file(GLOB containers Cuda/containers/*.cpp)

set(CUDA_ARCH_BIN "20 30 32 35 37 50 52" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
include(CudaComputeTargetFlags.cmake)                  
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")           
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false") 

CUDA_COMPILE(cuda_objs ${cuda})

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(libfreenect2_grabber 
               ${srcs}
               ${cuda} 
               ${cuda_objs} 
               ${containers}
)

target_link_libraries(libfreenect2_grabber
                      ${Boost_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${Eigen_LIBRARIES}
                      ${CUDA_LIBRARIES}
			${PCL_LIBRARIES})
