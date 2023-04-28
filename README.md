### Create a ROS workspace and go to the src folder

```
mkdir -p ~/ros_basics_ws/src && cd ~/ros_basics_ws/src
```

### Clone this GitHub repository into your src folder

```
git clone https://github.com/carlowiesse/ros_basics_activities.git
```

### Build ROS workspace:

```
source /opt/ros/humble/setup.bash

cd ~/ros_basics_ws
colcon build
```