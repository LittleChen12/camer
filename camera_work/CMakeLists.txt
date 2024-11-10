cmake_minimum_required(VERSION 3.5.0)
project(camera_work)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

## 2、设置c++编译器的配置 编译器的标准和行为
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

AUX_SOURCE_DIRECTORY(.  SRC_LIST)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src  SRC_LIST)
message(${SRC_LIST})

add_executable(camera_work_test ${SRC_LIST})
target_link_libraries(camera_work_test PRIVATE ${OpenCV_LIBS})