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

# find googletest library
find_package(GTest REQUIRED)
if(NOT GTest_FOUND)
  message(FATAL "Google Test library [gtest] not found.")
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

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/utils.cpp
  src/grid.cpp
  src/range_finder.cpp
  src/estimation.cpp
)

###########
## Tests ##
###########
add_subdirectory(tests)

################
## Excutables ##
################
enable_testing()
add_subdirectory(app)
