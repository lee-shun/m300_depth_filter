add_library(${PROJECT_NAME} DepthFilter.cpp)

add_executable(test_DepthFilter test_DepthFilter.cpp)
target_link_libraries(test_DepthFilter ${PROJECT_NAME} ${THIRD_PARTY_LIBS})

add_executable(test_SegmentLocationFinder test_SegmentLocationFinder.cpp)
target_link_libraries(test_SegmentLocationFinder ${THIRD_PARTY_LIBS})
