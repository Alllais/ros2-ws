# The `src/` directory

All your ROS2 packages should be in here.

ros2 launch uol_tidybot tidybot.launch.py
rosdep install -i --from-path src --rosdistro humble -y //check for missing dependencies
colcon build --packages-select py_pubsub //build the files
source install/setup.bash
ros2 run py_pubsub [talker / listener / {other topic name}] //running the files
ros2 topic echo /[topic]

setup.py has to be updated

to run camera feed "/bin/python3 /home/lcas/ros2-ws/src/py_pubsub/py_pubsub/openCV_Demo.py"

/bin/python3 /home/lcas/ros2-ws/src/py_pubsub/py_pubsub/color_chaser.py
