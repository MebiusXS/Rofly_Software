# rosbag record /drone_0_ego_planner_node/grid_map/occupancy_inflate /drone_0_ego_planner_node/goal_point /drone_0_ego_planner_node/global_list /drone_0_ego_planner_node/optimal_list /drone_0_ego_planner_node/failed_list /drone_0_ego_planner_node/init_list /vins_estimator/path /vins_estimator/imu_propagate /vins_estimator/image_track /mavros/imu/data_raw /setpoints_cmd
# rosbag record /drone_0_ego_planner_node/grid_map/occupancy_inflate /drone_0_ego_planner_node/goal_point /drone_0_ego_planner_node/global_list /drone_0_ego_planner_node/optimal_list /drone_0_ego_planner_node/failed_list /drone_0_ego_planner_node/init_list /mavros/imu/data_raw /setpoints_cmd /ekf/ekf_odom
rosbag record /debugPx4ctrl /mavros/imu/data /mavros/imu/data_raw /mavros/setpoint_raw/target_attitude /mavros/rc/out /traj_server/desired_position /Odometry /Odometry_imu /setpoints_cmd --tcpnodelay
