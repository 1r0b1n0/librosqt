cmake_minimum_required(VERSION 2.8.12)
project(librosqt)

find_package(catkin REQUIRED roscpp)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
find_package(Qt5Core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES librosqt
  CATKIN_DEPENDS roscpp
)

include_directories(include ${Qt5Core_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(librosqt src/QRosCallBackQueue.cpp include/librosqt/QRosCallBackQueue.h)

set(COMPILE_EXAMPLES off CACHE BOOL "Compile examples")

if(COMPILE_EXAMPLES)
    add_executable(simple_example examples/simple/main.cpp examples/simple/TestObject.cpp)
    target_link_libraries(simple_example
        librosqt
        ${catkin_LIBRARIES} Qt5::Core
    )

    add_executable(polling_example examples/alternatives/polling_method/main.cpp)
    target_link_libraries(polling_example
        ${catkin_LIBRARIES} Qt5::Core
    )

    add_executable(two_threads_example examples/alternatives/two_threads_method/main.cpp)
    target_link_libraries(two_threads_example
        ${catkin_LIBRARIES} Qt5::Core
    )
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS librosqt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
