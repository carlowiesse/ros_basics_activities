### Run nodes:

Terminal 1: *(Camera Driver)*
```
source /opt/ros/humble/setup.bash

cd ~/ros_basics_ws
source install/setup.bash
ros2 launch camera_driver cam.launch.py
```

Terminal 2: *(Camera Publisher)*
```
source /opt/ros/humble/setup.bash

cd ~/ros_basics_ws
source install/setup.bash
ros2 run camera_publisher image_processing_node
```

Terminal 3: *(Image Viewer)*
```
source /opt/ros/humble/setup.bash

ros2 run image_view image_view image:=/new_image
```