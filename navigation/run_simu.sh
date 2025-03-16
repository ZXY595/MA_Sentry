source ./install/setup.sh

#Notice: set mode -> nav 启用已知全局图模式
ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=mapping \
    lio:=pointlio \
    lio_rviz:=False \
    nav_rviz:=True
