message(STATUS "----------------------------------------------------------")
message(STATUS ${PROJECT_NAME} ": Running CMakeLists on: " ${CMAKE_CURRENT_SOURCE_DIR})

# This will allow you to quickly and simply add tests. 
macro(package_add_test TESTNAME FILES LIBRARIES)
  # create an exectuable in which the tests will be stored
  add_executable(${TESTNAME} ${FILES})
  # add the binary tree to the search path for include files
  # so that we will find include files
  target_include_directories(${TESTNAME} PUBLIC
                            "${CMAKE_CURRENT_BINARY_DIR}"
                            "${CMAKE_CURRENT_SOURCE_DIR}/include"
                            )
  # link the Google test infrastructure, mocking library, testing libraries and 
  # a default main fuction to the test executable.
  # Remove gtest_main if writing your own main function.
  target_link_libraries(${TESTNAME} gtest gtest_main pthread ${LIBRARIES} ${PROJECT_NAME})
  # gtest_discover_tests replaces gtest_add_tests,
  # see https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options to pass to it
  gtest_discover_tests(${TESTNAME}
      # set a working directory so your project root so that you can find test data via paths relative to the project root
      WORKING_DIRECTORY "${PROJECT_DIR}"
      PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${TEST_WORKING_DIRECTORY}"
  )
  # All users of this library will need at least C++14
  target_compile_features(${TESTNAME} PUBLIC cxx_std_14)
  set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()
# example of using the macro
# package_add_test(test1 test1.cpp lib_to_test)

###########
## Tests ##
###########

include_directories(${CMAKE_CURRENT_SOURCE_DIR} include ${gtest_SOURCE_DIR})

# Declare a C++ library
add_library(${PROJECT_NAME}_test
  include/${PROJECT_NAME}_test/helper.cpp
)

package_add_test(comm_tests 
  comm_utest.cpp
  ${PROJECT_NAME}_test)

package_add_test(utils_tests
  utils_utest.cpp
  ${PROJECT_NAME}_test)

package_add_test(rf_integ_test
  rf_itest.cpp
  ${PROJECT_NAME}_test)

package_add_test(grid_integ_test
  grid_itest.cpp
  ${PROJECT_NAME}_test)