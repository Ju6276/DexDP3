cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)
set(BUILD_SHARED_LIBS ON)
project(manus_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(
    include
    ManusSDK/include
)
catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
)

set(MANUS_LIBS ${CMAKE_SOURCE_DIR}/ManusSDK/lib/libManusSDK_Integrated.so)

add_executable(full_client
    src/main.cpp
    src/SDKClient.cpp
    src/ClientPlatformSpecific.cpp
)
set_target_properties(full_client PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags -Wl,--allow-shlib-undefined")
target_link_libraries(full_client ncurses pthread ${MANUS_LIBS} ${catkin_LIBRARIES})

add_dependencies(full_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
    full_client
    DESTINATION lib/${PROJECT_NAME}
)
