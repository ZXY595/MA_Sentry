source ./install/setup.sh

ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUC \
    mode:=mapping \
    lio:=pointlio \
    lio_rviz:=True \
    nav_rviz:=True
