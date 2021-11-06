cmake_minimum_required(VERSION 3.0.0)
project(ai-search-algorithms VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(OpenCV REQUIRED)
add_executable(ai-search-algorithms)
target_sources(ai-search-algorithms PRIVATE main.cpp Maze.cpp MazeGenerator.cpp)
target_link_libraries(ai-search-algorithms PRIVATE ${OpenCV_LIBS})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
