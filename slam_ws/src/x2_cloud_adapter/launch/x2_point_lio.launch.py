"""Point-LIO on the AgiBot X2 Ultra, with the cloud-format adapter in front."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz.')
    publish_arg = DeclareLaunchArgument(
        'publish_slam', default_value='false',
        description='Publish into the vendor slot (/slam/*). Off by default: this is '
                    'the only part of the stack that writes to the robot\'s bus.')

    cfg = PathJoinSubstitution(
        [FindPackageShare('x2_cloud_adapter'), 'config', 'x2_ultra.yaml'])

    adapter = Node(
        package='x2_cloud_adapter',
        executable='cloud_adapter',
        name='x2_cloud_adapter',
        output='screen',
        parameters=[{
            'input_topic': '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
            'output_topic': '/x2/points',
            'time_scale': 1.0e-9,     # vendor per-point timestamps are float64 ns
        }],
    )

    laser_mapping = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[cfg, {
            'use_imu_as_input': False,   # False = Point-LIO's own formulation
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            # [measured] the E1R is a 122.8 x 89.1 deg forward-facing sensor with
            # median range 3.4 m indoors. Decimating to 1-in-4 left only ~6.3k of
            # 25.2k points, too few constraints for a narrow FOV. Use them all.
            'point_filter_num': 1,
            'space_down_sample': True,
            'filter_size_surf': 0.25,    # [vendor] filter_size_surf
            'filter_size_map': 0.5,      # [vendor] filter_size_map
            'cube_side_length': 1000.0,  # [vendor] cube_side_length
            'runtime_pos_log_enable': False,
            # Point-LIO broadcasts camera_init->aft_mapped on /tf unconditionally
            # upstream. /tf is vendor-owned on this robot, so gate it off; set true
            # for offline/RViz use where the frames are wanted.
            'publish_tf': False,
        }],
    )

    # Relays our estimate into the vendor's vacant /slam/* slot. Measured on the
    # robot: every subscriber there is BEST_EFFORT/VOLATILE, and nothing publishes.
    slam_publisher = Node(
        package='x2_cloud_adapter',
        executable='slam_publisher',
        name='slam_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_slam')),
        parameters=[{
            'in_topic': '/aft_mapped_to_init',
            'localization_topic': '/slam/localization/odometry',
            'mapping_topic': '/slam/mapping/odometry',
            'lidar_odom_topic': '/slam/lidar_odom',
            # RELIABLE+TRANSIENT_LOCAL is the strictly most compatible writer; it
            # matches the BEST_EFFORT/VOLATILE readers measured on the robot.
            'reliable': True,
            'transient_local': True,
            'depth': 50,
            # Frames pass through (camera_init -> aft_mapped) unless set. What
            # soc0_task_manager expects is NOT yet confirmed -- see README.
            'header_frame_id': '',
            'child_frame_id': '',
            # /tf on the robot is vendor-owned; do not contend with it by default.
            'publish_tf': False,
        }],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('point_lio'), 'rviz_cfg', 'loam_livox.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
    )

    return LaunchDescription([rviz_arg, publish_arg, adapter, laser_mapping,
                              slam_publisher, rviz])
