include(UseGTest)

configure_gtest_directory(
    INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/..
    LINK_LIBRARIES vanetza)
add_gtest(PriorityRequestTable priority_request_table.cpp ../priority_request_table.cpp)
