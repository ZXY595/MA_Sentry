source ./install/setup.sh
ros2 topic pub /serial/receive2 rm_interfaces/msg/SerialReceiveData 'judge_system_data:
  game_status: 4
  hp: 200
  ammo: 200'
