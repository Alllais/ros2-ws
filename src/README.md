# The `src/` directory

All your ROS2 packages should be in here.

source install/setup.bash
colcon build --packages-select py_pubsub
ros2 run py_pubsub [talker / listener / {other topic name}]
ros2 topic echo /[topic]