cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller_simulation)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # Add other dependencies here
)

# Declare a catkin package
catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES mpc_controller_simulation
  CATKIN_DEPENDS roscpp rospy std_msgs
  # Add other dependencies here
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add executables
#add_executable(${PROJECT_NAME}_node src/mpc_node.py)
# Add Python scripts
catkin_install_python(PROGRAMS
      mpc_node.py
      mpc_controller.py
      plot_error.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#)

# Install targets
#install(TARGETS ${PROJECT_NAME}_node
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
