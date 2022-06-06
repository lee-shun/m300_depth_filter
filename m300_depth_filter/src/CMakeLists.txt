add_library(${PROJECT_NAME} SegmentLocationFinder.cpp DepthFilter.cpp
                            AbsolutePoseRecover.cpp)
target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY_LIBS})

add_executable(test_SegmentLocationFinder test_SegmentLocationFinder.cpp)
target_link_libraries(test_SegmentLocationFinder ${PROJECT_NAME})

add_executable(test_DepthFilter test_DepthFilter.cpp)
target_link_libraries(test_DepthFilter ${PROJECT_NAME})

add_executable(test_AbsolutePoseRecover test_AbsolutePoseRecover.cpp)
target_link_libraries(test_AbsolutePoseRecover ${PROJECT_NAME})
