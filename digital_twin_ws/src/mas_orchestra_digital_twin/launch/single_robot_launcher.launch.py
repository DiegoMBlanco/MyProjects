import os, xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import TimerAction
from launch_ros.actions import Node

robot_model = 'welding_arm' #welding_arm, gp7
robot_ns = 'r1' 
pose = ['0', '0', '0', '0'] # X, Y, Z, YAW ()
robot_base_color = '0.0 1.0 0.0 1.0' #Ign and Rviz color of the robot's main body 
world_file = 'empty.sdf' # empty

def generate_launch_description():

    this_pkg_path = os.path.join(get_package_share_directory('mas_orchestra_digital_twin'))

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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    open_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [this_pkg_path+"/worlds/"+world_file, ' -v 4', ' -r']) # ' -r'

        ]
    )

    xacro_file = os.path.join(this_pkg_path, 'urdf', robot_model+'.xacro') # o .urdf dependiendo del modelo

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
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', f'/{robot_ns}/controller_manager'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', f'/{robot_ns}/controller_manager'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    # Encadenar: gz_spawn_entity -> joint_state_broadcaster -> arm_controller
    """delay_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )"""

    # Retrasar 12 segundos para dar tiempo a Gazebo + gz_ros2_control
    """delay_jsb_spawner = TimerAction(
        period=12.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delay_arm_spawner = TimerAction(
        period=15.0,
        actions=[arm_controller_spawner],
    )"""


    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/world/world_model/model/'+robot_ns+'/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[   
            ('/world/world_model/model/'+robot_ns+'/joint_state', '/'+robot_ns+'/joint_states'),
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
            joint_state_broadcaster_spawner,   # 
            arm_controller_spawner,   # 
            bridge,
            Node(
                package = 'mas_orchestra_digital_twin',
                executable = 'tf_broadcaster',
                output = 'screen',
                arguments=[robot_ns]),
        ]
    )
