source ./install/setup.sh

ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=mapping \
    lio:=pointlio \
    lio_rviz:=False \
    nav_rviz:=True
