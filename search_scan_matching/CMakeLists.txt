# Works with 3.10.2
cmake_minimum_required(VERSION 3.10.2)

# Project name and settings
project(
  search_scan_matching
  VERSION 0.1
  DESCRIPTION "Search Scan Matching project"
  LANGUAGES CXX)

message(STATUS "----------------------------------------------------------")
message(STATUS ${PROJECT_NAME} ": Running CMakeLists on: " ${CMAKE_CURRENT_SOURCE_DIR})

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

  # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # Let's ensure -std=c++xx instead of -std=g++xx
  set(CMAKE_CXX_EXTENSIONS OFF)

  # Let's nicely support folders in IDEs
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  # Testing only available if this is the main app
  # Note this needs to be done in the main CMakeLists
  # since it calls enable_testing, which must be in the
  # main CMakeLists.
  include(CTest)

endif()

#########################
## System Dependencies ##
#########################

# FetchContent added in CMake 3.10, downloads during the configure step
# include(FetchContent)
# Set(FETCHCONTENT_QUIET TRUE)
# Set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)

# FetchContent_MakeAvailable was not added until CMake 3.11; use our shim
# if(${CMAKE_VERSION} VERSION_LESS 3.11)
#   include(cmake/add_FetchContent_MakeAvailable.cmake)
# endif()

# find googletest library
find_package(GTest REQUIRED)
if(NOT GTest_FOUND)
  message(FATAL_ERROR "Google Test library [gtest] not found. Fetch googletest")
  # googletest library
  # FetchContent_Declare(
  #   googletest
  #   GIT_REPOSITORY https://github.com/google/googletest.git
  #   GIT_TAG        release-1.8.0
  # )
  # FetchContent_MakeAvailable(googletest)
else()
  message(STATUS "[internal] FOUND Google Test library at ${GTEST}")
endif()

# find thread package
find_package(Threads REQUIRED)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
)

# link_libraries(
# )

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/common.cpp
  src/grid.cpp
  src/range_finder.cpp
  src/estimation.cpp
)
# # glob require CMake 3.12 or later:
# file(GLOB HEDERS_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

# source_group(
#   TREE "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}"
#   PREFIX "Header Files"
#   FILES ${HEDERS_LIST})


###########
## Tests ##
###########
add_subdirectory(tests)

################
## Excutables ##
################
add_subdirectory(app)

# create an exectuable
#  add_executable(agrid app/agrid.cpp src/grid.cpp)
#  target_include_directories(agrid PUBLIC
#                            "${PROJECT_BINARY_DIR}"
#                            "${PROJECT_SOURCE_DIR}/include"
#                            )
#  target_link_libraries(agrid PUBLIC ${PROJECT_NAME})

# The test code is here
# enable_testing()
# add_subdirectory(tests)
# include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# # 'test1.cpp tests2.cpp' are source files with tests
# add_executable(Google_Tests_run tests/utest.cpp)
# target_link_libraries(Google_Tests_run gtest gtest_main ${PROJECT_NAME})

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
#     include(CTest)
# endif()

# Mark executables for installation
# install(TARGETS manager_app
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# INSTALL(TARGETS ${PROJECT_NAME} 
#         LIBRARY DESTINATION some/libpath
#         PUBLIC_HEADER DESTINATION some/includepath
# )
