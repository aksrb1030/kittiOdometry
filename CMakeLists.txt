cmake_minimum_required(VERSION 3.2)
project(kittiOdometry)
set(CMAKE_BUILD_TYPE Release)

add_compile_options(-std=c++11)
#add_compile_options(-pg)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libLAS REQUIRED)
find_package(OpenGL REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${VTK_USE_FILE}
  ${PCL_INCLUDE_DIRS}
)

add_executable(kittiOdometry src/main.cpp src/kittiOdom.cpp)
target_link_libraries(kittiOdometry ${OpenCV_LIBS} ${VTK_LIBARRIES} ${PCL_LIBRARIES} /usr/lib/liblas.so.3
)
