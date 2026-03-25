cmake_minimum_required(VERSION 3.4)

project(motion_server CXX)

list(FILTER CMAKE_MODULE_PATH EXCLUDE REGEX "anaconda3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
  add_compile_options(/W0)
else()
  add_compile_options(-Wall -Wextra)
endif()

## Dependencies
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(REST_LIBS
  cpprest
  ssl
  crypto
  Boost::system
)

set(EIGEN_INC_DIR  ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN_INC_DIR})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(common STATIC
  common.cpp
)
target_link_libraries(common PRIVATE Franka::Franka)
target_include_directories(common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(motion_server motion_server.cpp)
target_include_directories(motion_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(motion_server
  Franka::Franka
  common
  ${REST_LIBS}
  Threads::Threads
)

include(GNUInstallDirs)
install(TARGETS motion_server
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
set(CMAKE_VERBOSE_MAKEFILE ON)
