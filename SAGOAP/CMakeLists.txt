# Collect all your source files
file(GLOB_RECURSE SAGOAP_SOURCES "*.cpp")

# Define the library (STATIC is easiest for local dev)
add_library(SAGOAP STATIC ${SAGOAP_SOURCES})

# Tell CMake where the header files are
# "PUBLIC" means projects that link to SAGOAP will also see these headers
target_include_directories(SAGOAP PUBLIC .)

