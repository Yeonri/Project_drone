include "map_builder.lua"
include "trajectory_builder.lua"

options = {

    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "base_link",
    published_frame = "base_link",
    odom_frame = "world",
    provide_odom_frame = true,
    publish_frame_projected_to_2d = true,
    use_odometry = false,
    use_nav_sat = false,
    use_landmarks = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,
    submap_publish_period_sec = 0.3,
    submap_publish_period_sec = 5e-3,
    trajectory_publish_period_sec = 30e-3,
    rangefinder_sampling_ratio = 1,
    odometry_sampling_ratio = 1,
    fixed_frame_pose_sampling_ratio = 1,
    imu_sampling_ratio = 1,
    landmarks_sampling_ratio = 1,
    pose_publish_period_sec = 5e-3
    
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1

POSE_GRAPH.optimization_problem.huber_scale = 5e1

-----------------TUNE THESE PARAMETERS FOR LOW LATENCY-------------------------------

------------Global SLAM------------
POSE_GRAPH.optimize_every_n_nodes = 90 -- Decrease origin: 100
POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
POSE_GRAPH.constraint_builder.min_score = 0.65 -- Increase
POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- Increase origin : 30

---------Global/Local SLAM---------
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50 -- Decrease
TRAJECTORY_BUILDER_2D.max_range = 6. -- Decrease


return options
