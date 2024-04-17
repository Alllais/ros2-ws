# The `src/` directory

The official name of the robot is "Boi vvall smol"

All your ROS2 packages should be in here.

add in a lidar and combine with a depth camera, lidar does not detect boxes so can be used for detecting walls and obstacles

environemnt setup
ros2 run  uol_tidybot generate_objects --ros-args -p red:=false -p n_objects:=10
ros2 launch uol_tidybot tidybot.launch.py
ros2 launch uol_tidybot tidybot.launch.py world:=level_2_1.world

rosdep install -i --from-path src --rosdistro humble -y //check for missing dependencies
colcon build --packages-select py_pubsub //build the files
source install/setup.bash
ros2 run py_pubsub [talker / listener / {other topic name}] //running the files
ros2 topic echo /[topic]

setup.py has to be updated

to run camera feed "/bin/python3 /home/lcas/ros2-ws/src/py_pubsub/py_pubsub/openCV_Demo.py"

/bin/python3 /home/lcas/ros2-ws/src/py_pubsub/py_pubsub/color_chaser.py
https://github.com/nonbinary-duck/ros2-tidybot-assignment/tree/main

limo/depth_camera_link/depth/image_raw

lower saturation for green
upper saturation for green

lower saturation for red
upper saturation for red

self.LOWER_LIGHTNESS  = 20;
self.UPPER_LIGHTNESS  = 255;
self.LOWER_SATURATION = 150;
self.UPPER_SATURATION = 255;
        
self.LOWER_RED_LOWER = np.array((0,   self.LOWER_SATURATION, self.LOWER_LIGHTNESS));
self.UPPER_RED_LOWER = np.array((20,  self.UPPER_SATURATION, self.UPPER_LIGHTNESS));

self.LOWER_RED_UPPER = np.array((235, self.LOWER_SATURATION, self.LOWER_LIGHTNESS));
self.UPPER_RED_UPPER = np.array((255, self.UPPER_SATURATION, self.UPPER_LIGHTNESS));

colour, depth and lidar camera

for c in contours

Check if the centre of that rectangle is above or below the centre of the image
isCube= bnd_centroid[1] > (cv_image.shape[0] * 0.5);

use this to check if its a cube or a banner