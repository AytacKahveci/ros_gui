cmake_minimum_required(VERSION 2.8.3)
project(rqt_program_mode)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  rosbag
  roscpp
  rospy
  std_msgs
  rqt_gui
  rqt_gui_cpp
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rqt_program_mode
  CATKIN_DEPENDS pluginlib rosbag roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

set(program_mode_SRCS
    src/program_mode.cpp)

set(program_mode_HDRS
    include/${PROJECT_NAME}/program_mode.h)

set(program_mode_UIS
    resources/launcher.ui)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(program_mode_MOCS ${program_mode_HDRS})
else()
  qt4_wrap_cpp(program_mode_MOCS ${program_mode_HDRS})
endif()

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(program_mode_UIS_H ${program_mode_UIS})
else()
  qt4_wrap_ui(program_mode_UIS_H ${program_mode_UIS})
endif()

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")
add_library(${PROJECT_NAME} ${program_mode_SRCS} ${program_mode_MOCS} ${program_mode_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


add_executable(parse_node src/parse.cpp)
target_link_libraries(parse_node -lpthread ${catkin_LIBRARIES})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)