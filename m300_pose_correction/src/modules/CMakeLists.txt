add_library(
  ${PROJECT_NAME}_modules
  SHARED
  ORBextractor.cc
  frame.cc
  viewer.cc
)

target_link_libraries(
  ${PROJECT_NAME}_modules
  ${THIRD_PARTY_LIBS}
)
