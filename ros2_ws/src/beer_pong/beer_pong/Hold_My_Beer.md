modidy velocity and accelerations:
 ros2 pkg prefix kinova_gen3_lite_moveit_config
 cd /opt/kortex_ws/install/kinova_gen3_lite_moveit_config/share/kinova_gen3_lite_moveit_config/config
 nano joint_limits.yaml

# 1 throw
ros2 run beer_pong beer_pong_throw --ros-args -p task:=throw_ball

# 6 throws (5 sec pause)
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball

# 3 throws
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball -p num_shots:=3

# 6 throws (2 sec pause)
ros2 run beer_pong beer_pong_throwall --ros-args -p task:=throw_ball -p pause_between:=2.0




