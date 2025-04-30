cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

#list(INSERT CMAKE_MODULE_PATH 0 "/opt/openrobots/lib/cmake/pinocchio")
#list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# remove from CMAKE_MODULE_PATH any path with anaconda3
list(FILTER CMAKE_MODULE_PATH EXCLUDE REGEX "anaconda3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
  add_compile_options(/W0)
else()
  add_compile_options(-Wall -Wextra)
endif()

set(THIRDPARTY_SOURCES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty" CACHE PATH
  "Directory for third-party sources")

## Dependencies

find_package(Franka REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/opt/openrobots/lib/cmake/casadi")
find_package(coal REQUIRED PATHS "/opt/openrobots/lib/cmake//coal")
find_package(casadi REQUIRED PATHS "/opt/openrobots/lib/cmake/casadi")
find_package(hpp-fcl REQUIRED PATHS "/opt/openrobots/lib/cmake//hpp-fcl")
find_package(pinocchio REQUIRED PATHS "/opt/openrobots/lib/cmake/pinocchio")
#find_package(pinocchio REQUIRED)

find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(REST_LIBS
  cpprest
  ssl
  crypto
  Boost::system
)

foreach(example ${EXAMPLES})
  target_link_libraries(${example} ${REST_LIBS})
endforeach()

set(EIGEN_INC_DIR  ${EIGEN3_INCLUDE_DIR})
set(INC_DIRS
    ${EIGEN_INC_DIR}
)
include_directories(${INC_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(common STATIC
  common.cpp
)

target_link_libraries(common PRIVATE Franka::Franka)
target_include_directories(common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set(EXAMPLES
  simple_motion
  motion_server
)
#joint_impedance_control
#franka_hello_world 
#grasp_object
#release_object


foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_include_directories(${example} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${example} Franka::Franka common pinocchio::pinocchio)
endforeach()

#target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_server ${REST_LIBS})
#if(TARGET motion_server)
#  message(STATUS "motion_server REST_LIBS: ${REST_LIBS}")
#  set_target_properties(motion_server PROPERTIES CXX_COMPILER g++)
#endif()

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
set(CMAKE_VERBOSE_MAKEFILE ON)
