echo "123" | sudo -S chmod 666 /dev/ttyACM0 #(串口赋权)
source ./install/setup.sh

#Notice: set mode -> nav 启用已知全局图模式
ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=libraryl2 \
    mode:=nav \
    localization:=slam_toolbox \
    lio:=pointlio \
    lio_rviz:=False \
    nav_rviz:=True
