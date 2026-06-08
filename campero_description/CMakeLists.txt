cmake_minimum_required(VERSION 3.5)
project(campero_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch meshes urdf ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/campero_description.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.flake8")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_campero_description test/test_campero_description.py)
endif()

ament_package()
