cmake_minimum_required(VERSION 3.20)
project(CampusPathEngineStudent VERSION 1.0.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(CPE_BUILD_APP "Build the raylib desktop application" ON)
option(CPE_BUILD_TESTS "Build headless core tests" ON)
option(CPE_ENABLE_SANITIZERS "Enable ASan/UBSan for GCC and Clang" OFF)

add_library(cpe_core STATIC
    src/engine/level.cpp
    src/engine/simulation.cpp
    src/engine/navigation_debug.cpp
    src/student/pathfinding.cpp
    src/student/navmesh.cpp
    src/student/navigation_agent.cpp
    src/student/enemy_ai.cpp
    src/student/gameplay.cpp
)
target_include_directories(cpe_core PUBLIC include)
if(MSVC)
    target_compile_options(cpe_core PRIVATE /W4 /utf-8)
else()
    target_compile_options(cpe_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(CPE_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(cpe_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(cpe_core PUBLIC -fsanitize=address,undefined)
endif()
if(CPE_BUILD_APP)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    # GLFW X11 is the supported Linux backend for this kit.
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
        set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
    endif()
    add_subdirectory(external/raylib EXCLUDE_FROM_ALL)
    add_executable(campus_path_engine src/engine/main.cpp)
    target_link_libraries(campus_path_engine PRIVATE cpe_core raylib)
    target_compile_definitions(campus_path_engine PRIVATE CPE_SOURCE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
    if(MSVC)
        target_compile_options(campus_path_engine PRIVATE /W4 /utf-8)
    else()
        target_compile_options(campus_path_engine PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    add_custom_command(TARGET campus_path_engine POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${CMAKE_CURRENT_SOURCE_DIR}/data" "$<TARGET_FILE_DIR:campus_path_engine>/data")
    install(TARGETS campus_path_engine RUNTIME DESTINATION .)
    install(DIRECTORY data DESTINATION .)
endif()
if(CPE_BUILD_TESTS)
    enable_testing()
    add_executable(cpe_tests tests/engine_tests.cpp)
    target_link_libraries(cpe_tests PRIVATE cpe_core)
    target_compile_definitions(cpe_tests PRIVATE CPE_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures" CPE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
    add_test(NAME cpe_core_tests COMMAND cpe_tests)
    add_executable(cpe_student_tests tests/student_contracts.cpp)
    target_link_libraries(cpe_student_tests PRIVATE cpe_core)
    add_test(NAME student_dijkstra COMMAND cpe_student_tests dijkstra)
    add_test(NAME student_astar COMMAND cpe_student_tests astar)
    add_test(NAME student_navmesh COMMAND cpe_student_tests navmesh)
    set_tests_properties(student_dijkstra student_astar student_navmesh
        PROPERTIES SKIP_RETURN_CODE 77)

endif()
