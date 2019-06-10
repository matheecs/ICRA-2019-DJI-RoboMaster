cmake_minimum_required(VERSION 3.1)
project(dwa)


aux_source_directory(src/.  SRC_LIST)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(
  ${CMAKE_CURRENT_SOURCE_DIR}/proto
  dwaProtoSrc
  dwaProtoHds
  ${ProtoFiles})

add_library(${PROJECT_NAME}
  SHARED
  ${dwaProtoSrc}
  ${dwaProtoHds}
  ${SRC_LIST}
  )

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${EXTERNAL_LIBS}
  )
add_library(planning::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
