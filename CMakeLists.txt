cmake_minimum_required(VERSION 3.15)
project(VR)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	tocabi_msgs
	cv_bridge
	image_transport
)
find_package(OpenCV REQUIRED
  NO_MODULE
  PATHS C:/Users/dyros/cv/opencv-3.4.13/build
  NO_DEFAULT_PATH
)
# Qt 5
# Important :
# Remember to set CMAKE_PREFIX_PATH to Qt5 cmake modules path for your targeted platform
# or set them here manually (not portable).
if(CMAKE_HOST_UNIX)
  list(APPEND CMAKE_PREFIX_PATH "/opt/Qt/5.15.2/gcc_64/lib/cmake")
else()
    if(${PLATFORM} MATCHES 64)
      set(suffix "_64")
    endif()
  list(APPEND DCMAKE_PREFIX_PATH "C:/Qt/5.15.2/msvc2019${suffix}/lib/cmake")
endif()

# Set CMake to run moc automatically has needed.
set(CMAKE_AUTOMOC ON)
# Set CMake to handle *.ui generation to avoid using QT5_WRAP_UI() macro.
set(CMAKE_AUTOUIC ON)
# Add the build dir to include paths to find UI's headers.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find Qt5 libraries.
foreach(module Core Gui Widgets)
    find_package(Qt5 REQUIRED COMPONENTS ${module})
    list(APPEND QT_INCLUDE_DIRS "${Qt5${module}_INCLUDE_DIRS}")
    list(APPEND QT_LIBRARIES    "${Qt5${module}_LIBRARIES}")
    list(APPEND QT_FLAGS        "${Qt5${module}_EXECUTABLE_COMPILE_FLAGS}")
    list(APPEND QT_DEFINITIONS  "${Qt5${module}_DEFINITIONS}")
endforeach()
# Correct QT_FLAGS having semicolons, which break the compile step on POSIX
list(REMOVE_DUPLICATES QT_FLAGS)

catkin_package( ## if you dont add this, executables are not found
  INCLUDE_DIRS include
#   LIBRARIES VR
  CATKIN_DEPENDS roscpp rospy std_msgs tocabi_msgs cv_bridge image_transport
)

add_executable(tracker src/main_tracker.cpp src/tracker.cpp)
add_executable(overlay src/main_overlay.cpp src/overlay.cpp src/overlay.h)
add_executable(overlay2 src/main_overlay2.cpp src/overlay2.cpp src/overlay.h)

include_directories(C:/Users/dyros/Desktop/avatar_ws/src/VR/include)

include_directories(C:/Users/dyros/Desktop/avatar_ws/src/VR/thirdparty/openvr/headers)
include_directories(C:/Users/dyros/Desktop/avatar_ws/src/VR/thirdparty/openvr/bin/win64)
include_directories(C:/Users/dyros/Desktop/avatar_ws/src/VR/thirdparty/eigen)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${QT_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(tracker ${catkin_LIBRARIES})
target_link_libraries(tracker C:/Users/dyros/Desktop/avatar_ws/src/VR/thirdparty/openvr/lib/win64/openvr_api.lib)

target_link_libraries(overlay ${catkin_LIBRARIES})
target_link_libraries(overlay ${OpenCV_LIBRARIES})
target_link_libraries(overlay ${QT_LIBRARIES})
target_link_libraries(overlay  C:/Users/dyros/Desktop/avatar_ws/src/VR/thirdparty/openvr/lib/win64/openvr_api.lib)

target_link_libraries(overlay2 ${catkin_LIBRARIES})
target_link_libraries(overlay2 ${OpenCV_LIBRARIES})
target_link_libraries(overlay2 ${QT_LIBRARIES})
target_link_libraries(overlay2  C:/Users/dyros/Desktop/avatar_ws/src/VR/thirdparty/openvr/lib/win64/openvr_api.lib)