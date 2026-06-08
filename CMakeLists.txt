cmake_minimum_required(VERSION 3.20)
project(swarm_sim VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

if(ENABLE_TSAN)
    add_compile_options(-fsanitize=thread)
    add_link_options(-fsanitize=thread)
endif()

# Include paths
include_directories(include)
include_directories(libs)

# ── Third-party ───────────────────────────────────

# cpp-httplib (header-only, requires threads)
find_package(Threads REQUIRED)
add_library(httplib INTERFACE)
target_include_directories(httplib INTERFACE libs/httplib)
target_link_libraries(httplib INTERFACE Threads::Threads)

# ── Binaries ──────────────────────────────────────

add_executable(drone
    src/drone/main.cpp
    src/drone/physics.cpp
)

add_executable(proxy
    src/proxy/main.cpp
    src/proxy/proxy.cpp
)

add_executable(gcs
    src/gcs/main.cpp
)
target_link_libraries(gcs PRIVATE httplib)

# ── Tests ─────────────────────────────────────────

enable_testing()

add_executable(tests_proxy
    tests/proxy/proxy_test.cpp
    src/drone/physics.cpp
    src/proxy/proxy.cpp
)

add_executable(tests_drone
    tests/drone/physics_test.cpp
    src/drone/physics.cpp
)

add_executable(tests_gcs
    tests/gcs/gcs_engine_test.cpp
    src/drone/physics.cpp
)
target_link_libraries(tests_gcs PRIVATE httplib)

add_executable(tests_common_net
    tests/common/net_test.cpp
)

add_test(NAME proxy_tests COMMAND tests_proxy)
add_test(NAME drone_tests COMMAND tests_drone)
add_test(NAME gcs_tests COMMAND tests_gcs)
add_test(NAME common_net_tests COMMAND tests_common_net)
