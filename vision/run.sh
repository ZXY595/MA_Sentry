echo "123" | sudo -S chmod 777 /dev/ttyACM0
source ./install/setup.bash
ros2 launch rm_bringup bringup.launch.py
#ros2 launch rm_bringup bringup_navigation.launch.py
