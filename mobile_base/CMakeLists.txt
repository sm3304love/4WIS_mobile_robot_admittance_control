cmake_minimum_required(VERSION 2.8.3)

project(mobile_base)

find_package(catkin REQUIRED COMPONENTS
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  std_msgs
  sensor_msgs
)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
