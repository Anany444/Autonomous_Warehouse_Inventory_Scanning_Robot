import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
import xacro

def generate_launch_description():
    
    # this name has to match the robot name in the Xacro file
    robotXacroName = 'warehouse_robot'
    
    # this is the name of our package, at the same time this is the name of the
    # folder that will be used to define the paths
    namePackageBringup = 'warehouse_robot_bringup'
    namePackageDescription = 'warehouse_robot_description'
    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'urdf/warehouse_robot.urdf'
    
    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackageDescription), modelFileRelativePath)
    
    # get the robot description from the xacro model file
    #robotDescription = xacro.process_file(pathModelFile).toxml()
    
    
    
    default_world = os.path.join(
        get_package_share_directory(namePackageDescription),
        'worlds',
        'warehouse.sdf'
        )
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'

        )
    
    slam_params = os.path.join(
        get_package_share_directory(namePackageBringup),
        'config',
        'mapper_params_online_async.yaml'
    )
    
    
    # this is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch', 'gz_sim.launch.py'))
        
    # this is if you are using an empty world model
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r  -v1 ', world], 'on_exit_shutdown':"true"}.items())
    
    # Robot State Publisher Node - MUST come before spawner
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ',pathModelFile])}, {'use_sim_time': True}]
    )
    
    # Gazebo node - spawn after robot_state_publisher is running
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robotXacroName,
            '-x', '-8.0',
            '-y', '-3.0',
            '-z', '0.0',
            '-Y', '1.57079632679',
        ],
        output='screen',
    )


    # Gazebo ROS Bridge Command Node
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
           "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            #"/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            #"/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output='screen',
        parameters=[
            {'use_sim_time':True}
        ]
    )
    
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )
    
    qr_scanner = Node(
        package='warehouse_mission_control',
        executable='qr_pipeline',
        name='qr_pipeline_node',
        output='screen',
    )
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    z_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['z_controller', '--controller-manager', '/controller_manager'],
    )
    
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )
    
    relay_cmd_vel = Node(
        package="topic_tools",
        executable="relay",
        name="relay_cmd_vel_to_diff_drive",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_controller/cmd_vel_unstamped",
            }
        ],
        output="screen",
    )
    
    slam=Node(package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[
               slam_params,
                {'use_sim_time': True},
             ],        
    )
    
    scan_matcher=Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        output='screen'
    )
    
    localisation=Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(namePackageBringup),'config','ekf.yaml')],
    )
    
    namespace_arg = DeclareLaunchArgument(
                        'namespace',
                        default_value='',
                        description='Robot namespace'
    )

    namespace = LaunchConfiguration('namespace')

    nav2 = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [get_package_share_directory( 'nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={'use_sim_time': 'True',
                              'use_robot_state_pub': 'True',
                              'use_composition': 'False',
                              'log_level': 'error',
                              'params_file':os.path.join(get_package_share_directory(namePackageBringup),'config','nav2_params.yaml')}.items()),
    ])
    Visualise= Node(
        package='warehouse_rack_detection',
        executable='visualise',
        name='map_visualizer_node',
        output='screen'
    )
    
    mission_executor=Node(
        package='warehouse_mission_control',
        executable='mission_executor',
        name='mission_executor_node',
        output='screen',
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory(namePackageBringup), 'config', 'rviz_config.rviz')]
    )
    
    # here we create an empty launch description object
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    # we add gazeboLaunch
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(ros_gz_image_bridge)
    launchDescriptionObject.add_action(qr_scanner)

    # Start robot_state_publisher first

    # Then spawn the robot after a delay
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(joint_state_broadcaster)
    launchDescriptionObject.add_action(z_controller)
    launchDescriptionObject.add_action(diff_drive_controller)
    launchDescriptionObject.add_action(relay_cmd_vel)
    launchDescriptionObject.add_action(scan_matcher)
    launchDescriptionObject.add_action(localisation)
    launchDescriptionObject.add_action(Visualise)

    launchDescriptionObject.add_action(slam)
    launchDescriptionObject.add_action(namespace_arg)
    launchDescriptionObject.add_action(nav2)
    launchDescriptionObject.add_action(mission_executor)
    launchDescriptionObject.add_action(rviz)
    
    return launchDescriptionObject
