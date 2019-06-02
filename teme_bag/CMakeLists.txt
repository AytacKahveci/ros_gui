cmake_minimum_required(VERSION 2.8.3)
project(teme_bag)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  qt_gui_cpp
  rqt_gui_cpp
  pluginlib
)

find_package(Boost REQUIRED COMPONENTS system filesystem)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

MESSAGE("qt_gui version" ${qt_gui_cpp_USE_QT_MAJOR_VERSION})
set(teme_bag_SRCS
  src/teme_bag.cpp)

set(teme_bag_HDRS
  include/teme_bag/teme_bag.h)

set(teme_bag_UIS
  resources/log_section.ui)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teme_bag
  CATKIN_DEPENDS rospy rqt_gui rqt_gui_cpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(teme_bag_MOCS ${teme_bag_HDRS})
else()
  qt4_wrap_cpp(teme_bag_MOCS ${teme_bag_HDRS})
endif()

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(teme_bag_UIS_H ${teme_bag_UIS})
else()
  qt4_wrap_ui(teme_bag_UIS_H ${teme_bag_UIS})
endif()

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")
add_library(${PROJECT_NAME} ${teme_bag_SRCS} ${teme_bag_MOCS} ${teme_bag_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

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