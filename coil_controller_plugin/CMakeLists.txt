cmake_minimum_required(VERSION 2.8.3)
project(coil_controller_plugin)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  python_qt_binding
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES coil_controller_plugin
  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/${PROJECT_NAME}/coil_controller_plugin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
