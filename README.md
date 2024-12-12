Build Project

```shell
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -o /usr/share/keyrings/ros-archive-keyring.gpg \
-sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools
sudo apt upgrade
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" \
>> ~/.bashrc
source ~/.bashrc
sudo rosdep init
rosdep update
```

```shell
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-gazebo-ros
```


```bash
rosdep install -i --from-path src --rosdistro jazzy -y
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select ur3_description
ros2 launch ur3_description ur3_rviz.launch.py
#ros2 launch ur3_description ur3_gazebo.launch.py
```