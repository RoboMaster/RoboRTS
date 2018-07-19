project(yolo)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

## Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--export-dynamic -Wall -Wno-sign-compare -fPIC")
endif()

#thread
find_package(Threads)
list(APPEND LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

# CUDA
find_package(CUDA)
if (CUDA_FOUND)
  message(STATUS "CUDA Version: " ${CUDA_VERSION_STRINGS})
  message(STATUS "CUDA Libararies: " ${CUDA_LIBRARIES})
  include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
  list(APPEND LIBRARIES ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES} ${CUDA_curand_LIBRARY} ${CUDA_cusparse_LIBRARY})
  list(APPEND CUDA_NVCC_FLAGS "-arch=sm_35;-O2;-Xcompiler;-fPIC;")
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  add_definitions(-DGPU)
  cuda_include_directories(src)
else()
  list(APPEND LIBRARIES "m")
endif()

#BOOST
find_package(Boost REQUIRED python)
find_package(PythonLibs REQUIRED)
find_package(OpenCV 3 REQUIRED)

if(OpenCV_FOUND)
  add_definitions(-DOPENCV)
endif()

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIR})

include_directories(src)
include_directories(examples)
set(SRC_DIR src)
set(EXAMPLE_DIR examples)

file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/${SRC_DIR}/*.c ${PROJECT_SOURCE_DIR}/${SRC_DIR}/*.cpp)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto ObjectDetectionProtoSrc ObjectDetectionProtoHds ${ProtoFiles})

if (CUDA_FOUND)

  file(GLOB CU_SRC_FILES ${PROJECT_SOURCE_DIR}/${SRC_DIR}/*.cu)

  CUDA_INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)

  set(CUDA_ARCH_BIN "62")
  set(CUDA_ARCH_PTX "62")

  include(CudaComputeTargetFlags)
  APPEND_TARGET_ARCH_FLAGS()

  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

  CUDA_COMPILE(cuda_objs ${CU_SRC_FILES})


  message(STATUS "CUDA FOUND")
endif()

add_library(dnl
  SHARED
  ${SRC_FILES}
  ${CU_SRC_FILES}
  ${ObjectDetectionProtoSrc}
  ${ObjectDetectionProtoHds}
  ${cuda_objs}
)
target_link_libraries(dnl
  PUBLIC
    driver::camera_param
    common::log
    ${LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  yolo_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  common::main_interface
  common::log
  common::io
  common::rrts
  dnl
)

add_executable(${PROJECT_NAME}_client
  yolo_client.cpp
)

target_link_libraries(${PROJECT_NAME}_client
  PRIVATE
  common::log
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARY}
)
