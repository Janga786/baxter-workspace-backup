cmake_minimum_required(VERSION 2.8.3)
project(fort_lewis_demo)
find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/baxter_demo_main.py
  scripts/baxter_face_expressions.py
  scripts/baxter_demo_utils.py
  scripts/baxter_demo_setup.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
