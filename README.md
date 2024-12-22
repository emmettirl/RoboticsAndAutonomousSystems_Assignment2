Build Project

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
  ros-jazzy-ros-gz-sim \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-gazebo-ros \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ur-robot-driver

sudo apt upgrade -y

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
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
colcon build --packages-select ur3_description --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 launch ur3_description ur3_gazebo.launch.py
```


Clean Installation
```shell
sudo rm -rf build install log
```