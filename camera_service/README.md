### Run nodes:

Terminal 1: *(Camera Driver)*
```
source /opt/ros/humble/setup.bash

cd ~/ros_basics_ws
source install/setup.bash
ros2 launch camera_driver cam.launch.py
```

Terminal 2: *(Camera Service)*
```
source /opt/ros/humble/setup.bash

cd ~/ros_basics_ws
source install/setup.bash
ros2 run camera_service aruco_detection_node
```

Terminal 3: *(Image Viewer)*
```
source /opt/ros/humble/setup.bash

ros2 run image_view image_view image:=/aruco_detection
```

Terminal 4: *(Service Request)*
```
source /opt/ros/humble/setup.bash

ros2 service call /trigger std_srvs/srv/Empty '{}'
```