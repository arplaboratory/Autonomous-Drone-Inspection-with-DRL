cmake_minimum_required(VERSION 2.8.3)
project(py_image_processor)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)

catkin_install_python(PROGRAMS scripts/image_processor.py scripts/processor.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
