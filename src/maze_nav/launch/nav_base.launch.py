from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='safe_drive'),
        DeclareLaunchArgument('map_source', default_value='yaml'),
        DeclareLaunchArgument('map_yaml', default_value='results/parte_a/casa_map_tuned.yaml'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('publish_loaded_map', default_value='true'),
        DeclareLaunchArgument('replan_on_map_update', default_value='false'),
        DeclareLaunchArgument('pose_topic', default_value='/odom'),
        DeclareLaunchArgument('pose_topic_type', default_value='odometry'),
        DeclareLaunchArgument('require_initial_pose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('inflation_radius_m', default_value='0.18'),
        DeclareLaunchArgument('path_stride_cells', default_value='1'),
        DeclareLaunchArgument('lookahead_m', default_value='0.20'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.06'),
        DeclareLaunchArgument('max_angular_rps', default_value='0.30'),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.09'),
        DeclareLaunchArgument('align_final_yaw', default_value='false'),
        DeclareLaunchArgument('obstacle_stop_distance_m', default_value='0.30'),
        DeclareLaunchArgument('emergency_stop_distance_m', default_value='0.16'),
        DeclareLaunchArgument('use_scan_obstacle_overlay', default_value='true'),
        DeclareLaunchArgument('scan_obstacle_stride', default_value='2'),
        DeclareLaunchArgument('scan_obstacle_min_cluster_size', default_value='3'),
        DeclareLaunchArgument('scan_obstacle_cluster_tolerance_m', default_value='0.25'),
        DeclareLaunchArgument('scan_obstacle_inflation_radius_m', default_value='0.22'),
        DeclareLaunchArgument('scan_obstacle_replan_period_s', default_value='1.0'),
        DeclareLaunchArgument('goal_progress_timeout_s', default_value='5.0'),
        DeclareLaunchArgument('goal_progress_epsilon_m', default_value='0.03'),
        DeclareLaunchArgument('goal_progress_min_motion_m', default_value='0.25'),
        DeclareLaunchArgument('goal_regression_timeout_s', default_value='12.0'),
        DeclareLaunchArgument('goal_regression_epsilon_m', default_value='0.04'),
        DeclareLaunchArgument('goal_regression_min_motion_m', default_value='0.45'),
        Node(
            package='maze_nav',
            executable='nav_base',
            name='maze_nav_base',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'mode': LaunchConfiguration('mode'),
                'map_source': LaunchConfiguration('map_source'),
                'map_yaml': LaunchConfiguration('map_yaml'),
                'map_topic': LaunchConfiguration('map_topic'),
                'publish_loaded_map': ParameterValue(
                    LaunchConfiguration('publish_loaded_map'),
                    value_type=bool,
                ),
                'replan_on_map_update': ParameterValue(
                    LaunchConfiguration('replan_on_map_update'),
                    value_type=bool,
                ),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'pose_topic_type': LaunchConfiguration('pose_topic_type'),
                'require_initial_pose': ParameterValue(
                    LaunchConfiguration('require_initial_pose'),
                    value_type=bool,
                ),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'inflation_radius_m': ParameterValue(
                    LaunchConfiguration('inflation_radius_m'),
                    value_type=float,
                ),
                'path_stride_cells': ParameterValue(
                    LaunchConfiguration('path_stride_cells'),
                    value_type=int,
                ),
                'lookahead_m': ParameterValue(
                    LaunchConfiguration('lookahead_m'),
                    value_type=float,
                ),
                'max_linear_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'),
                    value_type=float,
                ),
                'max_angular_rps': ParameterValue(
                    LaunchConfiguration('max_angular_rps'),
                    value_type=float,
                ),
                'goal_tolerance_m': ParameterValue(
                    LaunchConfiguration('goal_tolerance_m'),
                    value_type=float,
                ),
                'align_final_yaw': ParameterValue(
                    LaunchConfiguration('align_final_yaw'),
                    value_type=bool,
                ),
                'obstacle_stop_distance_m': ParameterValue(
                    LaunchConfiguration('obstacle_stop_distance_m'),
                    value_type=float,
                ),
                'emergency_stop_distance_m': ParameterValue(
                    LaunchConfiguration('emergency_stop_distance_m'),
                    value_type=float,
                ),
                'use_scan_obstacle_overlay': ParameterValue(
                    LaunchConfiguration('use_scan_obstacle_overlay'),
                    value_type=bool,
                ),
                'scan_obstacle_stride': ParameterValue(
                    LaunchConfiguration('scan_obstacle_stride'),
                    value_type=int,
                ),
                'scan_obstacle_min_cluster_size': ParameterValue(
                    LaunchConfiguration('scan_obstacle_min_cluster_size'),
                    value_type=int,
                ),
                'scan_obstacle_cluster_tolerance_m': ParameterValue(
                    LaunchConfiguration('scan_obstacle_cluster_tolerance_m'),
                    value_type=float,
                ),
                'scan_obstacle_inflation_radius_m': ParameterValue(
                    LaunchConfiguration('scan_obstacle_inflation_radius_m'),
                    value_type=float,
                ),
                'scan_obstacle_replan_period_s': ParameterValue(
                    LaunchConfiguration('scan_obstacle_replan_period_s'),
                    value_type=float,
                ),
                'goal_progress_timeout_s': ParameterValue(
                    LaunchConfiguration('goal_progress_timeout_s'),
                    value_type=float,
                ),
                'goal_progress_epsilon_m': ParameterValue(
                    LaunchConfiguration('goal_progress_epsilon_m'),
                    value_type=float,
                ),
                'goal_progress_min_motion_m': ParameterValue(
                    LaunchConfiguration('goal_progress_min_motion_m'),
                    value_type=float,
                ),
                'goal_regression_timeout_s': ParameterValue(
                    LaunchConfiguration('goal_regression_timeout_s'),
                    value_type=float,
                ),
                'goal_regression_epsilon_m': ParameterValue(
                    LaunchConfiguration('goal_regression_epsilon_m'),
                    value_type=float,
                ),
                'goal_regression_min_motion_m': ParameterValue(
                    LaunchConfiguration('goal_regression_min_motion_m'),
                    value_type=float,
                ),
            }],
        ),
    ])
