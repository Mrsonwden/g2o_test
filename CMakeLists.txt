cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH /home/snowden/tool/eigen-3.1.0/cmake)

project(g2o_test)

include_directories("/usr/local/include/eigen3")
include_directories(include)
include_directories(${G2O_INCLUDE_DIRS})
include_directories($(OpenCV_INCLUDE_DIRS))

find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(g2o_test src/main.cpp  src/g2o_test.cpp   src/generate_data.cpp)
target_link_libraries(g2o_test  g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension g2o_solver_csparse cholmod g2o_solver_cholmod  ${OpenCV_LIBS})
