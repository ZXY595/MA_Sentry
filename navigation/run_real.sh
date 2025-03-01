echo "123" | sudo -S chmod 666 /dev/ttyACM0 #(串口赋权)
source ./install/setup.sh

ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=RMUC \
    mode:=mapping  \
    localization:=slam_toolbox \
    lio:=pointlio \
    lio_rviz:=False \
    nav_rviz:=True
