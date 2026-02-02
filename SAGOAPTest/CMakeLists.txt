# Define the test executable
file(GLOB TEST_SOURCES "*.cpp")
add_executable(SAGOAP_TestRunner ${TEST_SOURCES})


target_link_libraries(SAGOAP_TestRunner PRIVATE SAGOAP)

target_link_libraries(SAGOAP_TestRunner PRIVATE 
    SAGOAP
    GTest::gtest_main
)

include(CTest)
add_test(NAME AllTests COMMAND SAGOAP_TestRunner)