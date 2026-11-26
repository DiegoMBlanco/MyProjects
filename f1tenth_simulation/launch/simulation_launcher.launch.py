import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

robot_model = 'f1tenth'
robot_ns = 'r1' 
pose = ['0.0', '0.0', '0.0', '0.0'] 
robot_base_color = '0.0 1.0 0.0 1.0' 
world_file = 'empty.sdf' 

def generate_launch_description():

    this_pkg_path = os.path.join(get_package_share_directory('f1tenth_simulation'))
  

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    # Set ign sim resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/rviz2_config.rviz")],
    )

    open_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [this_pkg_path+"/worlds/"+world_file, ' -v 4', ' -r']) # ' -r'

        ]
    )

    xacro_file = os.path.join(this_pkg_path, 'urdf', robot_model+'.xacro') 

    doc = xacro.process_file(xacro_file,
        mappings={'base_color' : robot_base_color, 'ns' : robot_ns})

    robot_desc = doc.toprettyxml(indent='  ')
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', pose[0], '-y', pose[1], '-z', pose[2],
                   '-R', '0.0', '-P', '0.0', '-Y', pose[3],
                   '-name', robot_ns,
                   '-allow_renaming', 'false'],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns,
        output="screen",
        parameters=[{'robot_description': robot_desc}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/model/'+robot_ns+'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/world/world_model/model/'+robot_ns+'/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/'+robot_ns+'/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/world_model/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/'+robot_ns+'/hokuyo@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/'+robot_ns+'/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/'+robot_ns+'/zed_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/'+robot_ns+'/zed_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        parameters=[{'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[   
            ('/model/'+robot_ns+'/cmd_vel', '/'+robot_ns+'/cmd_vel'),
            ('/world/world_model/model/'+robot_ns+'/joint_state', '/'+robot_ns+'/joint_states'),
            ('/model/'+robot_ns+'/odometry', '/'+robot_ns+'/odom'),
            ('/'+robot_ns+'/hokuyo', '/'+robot_ns+'/hokuyo'),
            ('/'+robot_ns+'/imu', '/'+robot_ns+'/imu'),
            ('/'+robot_ns+'/zed_camera/image', '/'+robot_ns+'/zed_camera/image'),
            ('/'+robot_ns+'/zed_camera/points', '/'+robot_ns+'/zed_camera/points'),
        ]
    )
    
    return LaunchDescription(
        [
            simu_time,
            ign_resource_path,
            open_rviz,
            open_ign,
            gz_spawn_entity,
            robot_state_publisher,
            bridge,
            Node(
                package = 'f1tenth_simulation',
                executable = 'tf_broadcaster',
                output = 'screen',
                arguments=['r1']),
            Node(
                package = 'f1tenth_simulation',
                executable = 'ground_truth_broadcaster',
                output = 'screen'),
            
        ]
    )