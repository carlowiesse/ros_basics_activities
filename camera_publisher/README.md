### Create a ROS workspace and go to the src folder

```
mkdir -p ~/ros2_basics_ws/src && cd ~/ros2_basics_ws/src
```

### Clone this GitHub repository into your src folder

```
git clone https://github.com/carlowiesse/ros2_basics_ex1.git ex1
```

### Build ROS workspace:

```
source /opt/ros/humble/setup.bash

cd ~/ros2_basics_ws
colcon build
```

### Run nodes:

Terminal 1: *(Camera Driver)*
```
source /opt/ros/humble/setup.bash

cd ~/ros2_basics_ws
source install/setup.bash
ros2 launch cam_launch_pkg cam.launch.py
```

Terminal 2: *(Image Publisher)*
```
source /opt/ros/humble/setup.bash

cd ~/ros2_basics_ws
source install/setup.bash
ros2 run image_processing node
```

Terminal 3: *(Camera Viewer)*
```
source /opt/ros/humble/setup.bash

ros2 run image_view image_view image:=/camera/image_raw
```

Terminal 4: *(Image Viewer)*
```
source /opt/ros/humble/setup.bash

ros2 run image_view image_view image:=/new_image
```