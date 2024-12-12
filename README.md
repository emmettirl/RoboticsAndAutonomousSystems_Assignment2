Build Project

```bash
source ~/ros2_ws/install/setup.bash
colcon build --symlink-install
ros2 launch ur3_description ur3_gazebo.launch.py
```