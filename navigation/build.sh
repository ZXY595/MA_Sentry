# colcon build --symlink-install
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
colcon build --parallel-workers 4
