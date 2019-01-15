project(io)

#common::io
add_library(io INTERFACE)

target_sources(io INTERFACE io.h)
target_link_libraries(io INTERFACE ${PROTOBUF_LIBRARIES})
