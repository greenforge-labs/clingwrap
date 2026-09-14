cmake_minimum_required(VERSION 3.8)
project(clingwrap)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_topic_throttle tests/test_topic_throttle.py ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
