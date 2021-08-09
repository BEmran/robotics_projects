message(STATUS "----------------------------------------------------------")
message(STATUS ${PROJECT_NAME} ": Running CMakeLists on: " ${CMAKE_CURRENT_SOURCE_DIR})

##########################################################################################
# This will allow you to quickly and simply add executable. 
macro(add_exec EXECNAME)
  # parse variables
  cmake_parse_arguments(PREFIX
                        "" # options
                        "" # single value
                        "FILES;LIBRARIES" # multi-value
                        ${ARGN}
                        )
  # create an exectuable
  add_executable(${EXECNAME} ${PREFIX_FILES})
  # add the binary tree to the search path for include files
  # so that we will find include files
  target_include_directories(${EXECNAME} PUBLIC
                            "${PROJECT_BINARY_DIR}"
                            "${PROJECT_SOURCE_DIR}/include"
                            )
  # link libraries
  target_link_libraries(${EXECNAME} PUBLIC ${PREFIX_LIBRARIES})
  # All users of this library will need at least C++14
  target_compile_features(${EXECNAME} PUBLIC cxx_std_14)
endmacro()
# example of using the macro
# add_exec(test1 FILES test1.cpp LIBRARIES lib_to_test)
##########################################################################################

# create executable for mpu_exec
add_exec(main
         FILES
         main.cpp
         LIBRARIES
         ${PROJECT_NAME}
         )