# hmw3

to launch the manipulator:

	ros2 launch iiwa_bringup iiwa.launch.py use_vision:=true

to build the camera bridge:

	ros2 run ros_ign_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image

