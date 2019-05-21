# Finds Google Protocol Buffers library and compilers and extends
# the standard cmake script

find_package(Protobuf REQUIRED)
list(APPEND RRTS_INCLUDE_DIRS PUBLIC ${PROTOBUF_INCLUDE_DIR})
list(APPEND RRTS_LINKER_LIBS PUBLIC ${PROTOBUF_LIBRARIES})
add_definitions(-DBUILD_PROTO)
#set(BUILD_PROTO false)

# As of Ubuntu 14.04 protoc is no longer a part of libprotobuf-dev package
# and should be installed separately as in: sudo apt-get install protobuf-compiler
if(EXISTS ${PROTOBUF_PROTOC_EXECUTABLE})
  message(STATUS "Found PROTOBUF Compiler: ${PROTOBUF_PROTOC_EXECUTABLE}")
else()
  message(FATAL_ERROR "Could not find PROTOBUF Compiler")
endif()

if (PROTOBUF_FOUND)
  message ("protobuf ${PROTOBUF_LIBRARIES} found")
else ()
  message (FATAL_ERROR "Cannot find protobuf")
endif ()

# place where to generate protobuf sources
#set(proto_gen_folder "${PROJECT_BINARY_DIR}/include/caffe/proto")
#include_directories("${PROJECT_BINARY_DIR}/include")

set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)

################################################################################################
# Modification of standard 'protobuf_generate_cpp()' with output dir parameter
# Usage:
#   rrts_protobuf_generate_cpp(<output_dir> <proto_srcs> <proto_hdrs> <proto_files>)
function(rrts_protobuf_generate_cpp output_dir proto_srcs proto_hdrs)
  if(NOT ARGN)
    message(SEND_ERROR "Error: rrts_protobuf_generate_cpp() called without any proto files")
    return()
  endif()

  set(${proto_srcs})
  set(${proto_hdrs})

  message("${BUILD_PROTO}")
  if (NOT ${BUILD_PROTO})
    foreach(fil ${ARGN})
      get_filename_component(abs_fil ${fil} ABSOLUTE)
      get_filename_component(fil_we ${fil} NAME_WE)
      list(APPEND ${proto_srcs} "${output_dir}/${fil_we}.pb.cc")
      list(APPEND ${proto_hdrs} "${output_dir}/${fil_we}.pb.h")
    endforeach()
    set_source_files_properties(${${proto_srcs}} ${${proto_hdrs}} PROPERTIES GENERATED TRUE)
    set(${proto_srcs} ${${proto_srcs}} PARENT_SCOPE)
    set(${proto_hdrs} ${${proto_hdrs}} PARENT_SCOPE)
    return()
  endif ()

  if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
    # Create an include path for each file specified
    foreach(fil ${ARGN})
      get_filename_component(abs_fil ${fil} ABSOLUTE)
      get_filename_component(abs_path ${abs_fil} PATH)
      list(FIND _protoc_include ${abs_path} _contains_already)
      if(${_contains_already} EQUAL -1)
        list(APPEND _protoc_include -I ${abs_path})
      endif()
    endforeach()
  else()
    set(_protoc_include -I ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(dir ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(abs_path ${dir} ABSOLUTE)
      list(FIND _protoc_include ${abs_path} _contains_already)
      if(${_contains_already} EQUAL -1)
        list(APPEND _protoc_include -I ${abs_path})
      endif()
    endforeach()
  endif()

  foreach(fil ${ARGN})
    get_filename_component(abs_fil ${fil} ABSOLUTE)
    get_filename_component(fil_we ${fil} NAME_WE)
    
    list(APPEND ${proto_srcs} "${output_dir}/${fil_we}.pb.cc")
    list(APPEND ${proto_hdrs} "${output_dir}/${fil_we}.pb.h")

    add_custom_command(
      OUTPUT "${output_dir}/${fil_we}.pb.cpp"
             "${output_dir}/${fil_we}.pb.h"
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out ${output_dir} ${_protoc_include} ${abs_fil}
      DEPENDS ${abs_fil}
      COMMENT "Running C++ protocol buffer compiler on ${fil}"
      VERBATIM )
  endforeach()

  set_source_files_properties(${${proto_srcs}} ${${proto_hdrs}} PROPERTIES GENERATED TRUE)
  set(${proto_srcs} ${${proto_srcs}} PARENT_SCOPE)
  set(${proto_hdrs} ${${proto_hdrs}} PARENT_SCOPE)
endfunction()
