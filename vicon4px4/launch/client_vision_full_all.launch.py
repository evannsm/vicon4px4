from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch both full_state_relay and optitrack_px4_relay with Vicon parameters."""
    
    ld = LaunchDescription()

    # ============================================================================
    # Declare launch arguments
    # ============================================================================
    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.10.2',
        description='Vicon server hostname or IP address'
    )
    buffer_size_arg = DeclareLaunchArgument(
        'buffer_size',
        default_value='200',
        description='Buffer size for Vicon data'
    )
    topic_namespace_arg = DeclareLaunchArgument(
        'topic_namespace',
        default_value='vicon',
        description='Topic namespace for Vicon messages'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='map',
        description='World frame for the tf2 transformations'
    )
    vicon_frame_arg = DeclareLaunchArgument(
        'vicon_frame',
        default_value='vicon',
        description='Vicon frame for the tf2 transformations'
    )
    map_xyz_arg = DeclareLaunchArgument(
        'map_xyz',
        default_value='[0.0, 0.0, 0.0]',
        description='XYZ translation for coordinate frame mapping'
    )
    map_rpy_arg = DeclareLaunchArgument(
        'map_rpy',
        default_value='[0.0, 0.0, 0.0]',
        description='RPY rotation for coordinate frame mapping'
    )
    map_rpy_in_degrees_arg = DeclareLaunchArgument(
        'map_rpy_in_degrees',
        default_value='false',
        description='Whether RPY values are in degrees (true) or radians (false)'
    )

    # Add all launch arguments to LaunchDescription
    ld.add_action(hostname_arg)
    ld.add_action(buffer_size_arg)
    ld.add_action(topic_namespace_arg)
    ld.add_action(world_frame_arg)
    ld.add_action(vicon_frame_arg)
    ld.add_action(map_xyz_arg)
    ld.add_action(map_rpy_arg)
    ld.add_action(map_rpy_in_degrees_arg)

    # ============================================================================
    # Get launch configuration values
    # ============================================================================
    hostname = LaunchConfiguration('hostname')
    buffer_size = LaunchConfiguration('buffer_size')
    topic_namespace = LaunchConfiguration('topic_namespace')
    world_frame = LaunchConfiguration('world_frame')
    vicon_frame = LaunchConfiguration('vicon_frame')
    map_xyz = LaunchConfiguration('map_xyz')
    map_rpy = LaunchConfiguration('map_rpy')
    map_rpy_in_degrees = LaunchConfiguration('map_rpy_in_degrees')

    # ============================================================================
    # Define nodes
    # ============================================================================


    # Node 1: Vicon client with parameters
    vicon_client = Node(
        package='vicon4px4',
        executable='vicon_client',
        output='screen',
        parameters=[{
            'hostname': hostname,
            'buffer_size': buffer_size,
            'namespace': topic_namespace,
            'world_frame': world_frame,
            'vicon_frame': vicon_frame,
            'map_xyz': map_xyz,
            'map_rpy': map_rpy,
            'map_rpy_in_degrees': map_rpy_in_degrees
        }]
    )

    # Node 2: Visual odometry relay (from mocap_px4_relays)
    visual_odometry_relay = Node(
        package='mocap_px4_relays',
        executable='visual_odometry_relay',
        output='screen',
    )

    # Node 3: Full state relay (from mocap_px4_relays)
    full_state_relay = Node(
        package='mocap_px4_relays',
        executable='full_state_relay',
        output='screen',
    )

    # Add nodes to LaunchDescription
    ld.add_action(vicon_client)
    ld.add_action(visual_odometry_relay)
    ld.add_action(full_state_relay)

    return ld