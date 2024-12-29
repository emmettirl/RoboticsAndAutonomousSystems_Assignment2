[//]: # (Installation and Configuration)
[//]: # (```shell)
[//]: # (sudo apt update && sudo apt install -y \ )
[//]: # (  software-properties-common \ )
[//]: # (  curl)
[//]: # ()
[//]: # (sudo add-apt-repository universe)
[//]: # ()
[//]: # (sudo curl -o /usr/share/keyrings/ros-archive-keyring.gpg -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key)
[//]: # (echo "deb [arch=$&#40;dpkg --print-architecture&#41; signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $&#40;. /etc/os-release && echo $UBUNTU_CODENAME&#41; main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null)
[//]: # ()
[//]: # (sudo apt update && sudo apt install -y \ )
[//]: # (  ros-dev-tools \ )
[//]: # (  python3-colcon-common-extensions \ )
[//]: # (  ros-jazzy-desktop \ )
[//]: # (  ros-jazzy-xacro \ )
[//]: # (  ros-jazzy-ros-gz-sim \ )
[//]: # (  ros-jazzy-joint-state-publisher-gui \ )
[//]: # (  ros-jazzy-gazebo-ros \ )
[//]: # (  ros-jazzy-ros2-control \ )
[//]: # (  ros-jazzy-ros2-controllers \ )
[//]: # (  ros-jazzy-ur-robot-driver \ )
[//]: # (  ros-jazzy-gz-ros2-control \ )
[//]: # (  ros-jazzy-ros-gz-sim \ )
[//]: # (  ros-jazzy-ros-gz \ )
[//]: # (  )
[//]: # ()
[//]: # (sudo apt upgrade -y)
[//]: # ()
[//]: # (echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc)
[//]: # (echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc)
[//]: # (echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc)
[//]: # (echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc)
[//]: # (echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> ~/.bashrc)
[//]: # (source ~/.bashrc)
[//]: # (```)


[//]: # (Make Package)
[//]: # (```shell)
[//]: # (cd ~/ros2_ws/src)
[//]: # (ros2 pkg create --build-type ament_cmake ur3_description)
[//]: # (ros2 pkg create --build-type ament_cmake ur3_move)
[//]: # (```)


[//]: # (Rosdep)
[//]: # (``` bash)
[//]: # (sudo rosdep init)
[//]: # (rosdep update)
[//]: # (rosdep install -i --from-path src --rosdistro jazzy -y)
[//]: # (```)


[//]: # (Run RVIZ)
[//]: # (```bash)
[//]: # (source ~/ros2_ws/install/setup.bash)
[//]: # (ros2 launch ur3_description ur3_rviz.launch.py)
[//]: # (```)


Clean Installation
```shell
sudo rm -rf build install log
```
Used to wipe the Build Install and Log directories in the current directory to start fresh.


Build Project
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_description ur3_move --symlink-install
```
Source the ROS2 setup.bash file, then build the project using colcon build.

Run Gazebo
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur3_description ur3_gazebo.launch.py
```
Source the ROS2 setup.bash file, then launch the ur3_gazebo.launch.py file.


Run Move
```bash
source ~/ros2_ws/install/setup.bash

#ros2 run ur3_move userCommandNode.py
#ros2 run ur3_move userInputNode.py
#ros2 run ur3_move UR3MoveNode.py
#ros2 run ur3_move UR3MoveActionClient.py
ros2 run ur3_move UR3RLEnvironment.py
```
Source the ROS2 setup.bash file, then run the desired node. Only one of the above should be run at a time. Comment out any others.

UR3MoveNode.py requires the joint_trajectory_controller_spawner to be disabled in ur3_gazebo.launch.py

UR3MoveActionClient.py # requires the robot_controller_spawner to be disabled in ur3_gazebo.launch.py 

This is done by comment out the controller spawner in the launch file, and commenting the name out of the list of Nodes.

[ur3_gazebo.launch.py](src/ur3_description/launch/ur3_gazebo.launch.py)



[//]: # (Misc troubleshooting and testing)

[//]: # (check for controller)
[//]: # (```shell)
[//]: # (ros2 node list)
[//]: # (ros2 control list_hardware_interfaces)
[//]: # (ros2 control list_controllers)
[//]: # (```)


[//]: # (test movement)
[//]: # (```shell)
[//]: # (ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5, 0.5, -0.5, 0.5]")
[//]: # (```)

