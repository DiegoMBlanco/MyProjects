import os
import xacro
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# ============================================================
# CONFIGURATION
# ============================================================

robot_model = 'welding_arm'  # welding_arm, gp7

robot_base_color = '0.0 1.0 0.0 1.0'

world_file = 'factory.world' # empty.sdf, small_warehouse.world, factory.world

# namespace : (X, Y, Z, YAW)
robots = {
    'r1',
    'r2',
    'r3',
    'r4',
    'r5',
    'r6',
}


def generate_launch_description():

    this_pkg_path = os.path.join(
        get_package_share_directory(
            'mas_orchestra_digital_twin'
        )
    )

    xacro_file = os.path.join(
        this_pkg_path,
        'urdf',
        'multi_welding_arm.xacro'
    )

    doc = xacro.process_file(xacro_file)
    
    robot_desc = doc.toprettyxml(indent='  ')

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',

        arguments=[
            '-string',
            robot_desc,
            
            '-name',
            'welding_cell',

            '-allow_renaming',
            'false'
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        name='robot_state_publisher',

        output='screen',

        parameters=[
            {
                'robot_description': robot_desc,
                'use_sim_time':
                LaunchConfiguration('use_sim_time')
            }
        ]
    )



    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )


    # GAZEBO RESOURCE PATH
    """ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'),
            ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )"""

    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'),
            ':',
            this_pkg_path,
            ':',
            os.path.expanduser(
                '~/MAS-ORCHESTRA/digital_twin_ws/src'
            )
        ]
    )


    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',os.path.join(this_pkg_path,'rviz','rviz2_config.rviz')
        ],
        parameters=[
            {
                'use_sim_time':
                LaunchConfiguration('use_sim_time')
            }
        ],
    )


    open_ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')
        ),
        launch_arguments=[
            ('gz_args',[os.path.join(this_pkg_path,'worlds',world_file),' -v 4',' -r'])
        ]
    )

    ld = LaunchDescription()

    ld.add_action(simu_time)
    ld.add_action(ign_resource_path)
    ld.add_action(open_ign)
    ld.add_action(open_rviz)
    ld.add_action(robot_state_publisher)
    ld.add_action(gz_spawn_entity)


    for robot_ns in robots:
        print(f'Launching robot {robot_ns} ')


        
    tf_broadcaster = Node(
        package='mas_orchestra_digital_twin',
        executable='tf_broadcaster',

        output='screen',

        arguments=[
            robot_ns
        ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],

        parameters=[
            {
                'use_sim_time':
                LaunchConfiguration('use_sim_time')
            }
        ],

        output='screen',
    )


    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],

        parameters=[
            {
                'use_sim_time':
                LaunchConfiguration('use_sim_time')
            }
        ],

        output='screen',
    )

    delay_jsb_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner
            ],
        )
    )

    delay_arm_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_controller_spawner
            ],
        )
    )

    ld.add_action(delay_jsb_spawner)
    ld.add_action(delay_arm_spawner)
    #ld.add_action(tf_broadcaster)


    return ld