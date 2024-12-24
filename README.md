Installation and Configuration
```shell
sudo apt update && sudo apt install -y \
  software-properties-common \
  curl

sudo add-apt-repository universe

sudo curl -o /usr/share/keyrings/ros-archive-keyring.gpg -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y \
  ros-dev-tools \
  python3-colcon-common-extensions \
  ros-jazzy-desktop \
  ros-jazzy-xacro \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-gazebo-ros \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ur-robot-driver \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim \
  

sudo apt upgrade -y

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

Make Package
```shell
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake ur3_description
ros2 pkg create --build-type ament_cmake ur3_move
```


``` bash
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
```


Run RVIZ
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 launch ur3_description ur3_rviz.launch.py
```

Run Gazebo
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_description ur3_move --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 launch ur3_description ur3_gazebo.launch.py
```

Run Move
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur3_move userCommandNode.launch.py
```

Clean Installation
```shell
sudo rm -rf build install log
```

check for controller
```shell
ros2 node list
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

test movement
```shell
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5, 0.5, -0.5, 0.5]"
```

