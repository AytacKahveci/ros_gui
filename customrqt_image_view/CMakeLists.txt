cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(customrqt_image_view)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp image_transport sensor_msgs geometry_msgs cv_bridge)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(customrqt_image_view_SRCS
  src/customrqt_image_view/image_view.cpp
  src/customrqt_image_view/ratio_layouted_frame.cpp
)

set(customrqt_image_view_HDRS
  include/customrqt_image_view/image_view.h
  include/customrqt_image_view/ratio_layouted_frame.h
)

set(customrqt_image_view_UIS
  src/customrqt_image_view/image_view.ui
)

set(customrqt_image_view_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${customrqt_image_view_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge geometry_msgs
)
#catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(customrqt_image_view_MOCS ${customrqt_image_view_HDRS})
else()
  qt4_wrap_cpp(customrqt_image_view_MOCS ${customrqt_image_view_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(customrqt_image_view_UIS_H ${customrqt_image_view_UIS})
else()
  qt4_wrap_ui(customrqt_image_view_UIS_H ${customrqt_image_view_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${customrqt_image_view_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${customrqt_image_view_SRCS} ${customrqt_image_view_MOCS} ${customrqt_image_view_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

message("Lib Destination: " ${CATKIN_PACKAGE_LIB_DESTINATION})
message("Share Destination: " ${CATKIN_PACKAGE_SHARE_DESTINATION})

#catkin_install_python(PROGRAMS scripts/rqt_image_view
#  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
#catkin_install_python(PROGRAMS scripts/rqt_image_view
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
