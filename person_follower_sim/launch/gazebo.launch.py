# #! /usr/bin/env python3
# import os
# import xacro
# from os.path import join
# from launch.actions import DeclareLaunchArgument
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import LaunchConfiguration, Command, PythonExpression
# import launch.conditions
# from launch_ros.descriptions import ParameterValue

# def generate_launch_description():
    
#     #path to xacro file
#     pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
#     my_pkg_dir = get_package_share_directory('person_follower_sim')
#     # world_path = my_pkg_dir + "/worlds/" + "model.sdf"
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     bridge_config = os.path.join('my_pkg_dir', 'config', 'bridge.yaml')
#     actor_world1 = join(get_package_share_directory("person_follower_sim"),'worlds','walk.world')


#     # actor_world = get_package_share_directory("person_follower_sim"),'worlds','stand.world')
#     # actor_world1 = get_package_share_directory("person_follower_sim"),'worlds','walk.world')
 

#     # # Include the gazebo.launch.py file
#     # gazebo=IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
#     #     launch_arguments={'pause': 'true',
#     #                       'world':actor_world1}.items()
#     # )

#     # declare_world_arg = DeclareLaunchArgument('world', default_value= world_path, description='name of the world to launch')



#    # launch GZ Sim with empty world
#     SetEnvironmentVariable(
#             name="GZ_SIM_RESOURCE_PATH",
#             value=my_pkg_dir
#         ),
    
#     gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
#         ),
#           launch_arguments={
#             'gz_args' :  actor_world1 + " -v 4"
#         }.items()  
        
#     )

#     # Declare launch argument for simulation time
#     arg_use_sim_time = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description="Enable sim time from /clock"
#     )


#     #spawn world
#     world=Node(
#         package='gazebo_ros',
#         executable='create',
#         name='spawn_model',
#         output='screen',
#         arguments=['-entity', 'first_world', '-file', my_pkg_dir],
#     )

#     # #publishing robot_state into topic robot_description
#     # robot_state=Node(package = 'robot_state_publisher',
#     #                         executable = 'robot_state_publisher',
#     #                         name='robot_state_publisher',
#     #                         parameters = [{'robot_description': ParameterValue(Command( \
#     #                                     ['xacro ', xacro_file,
#     #                                     ' kinect_enabled:=', "true",
#     #                                     # ' lidar_enabled:=', lidar_enabled,
#     #                                     # ' camera_enabled:=', camera_enabled,
#     #                                     ]), value_type=str)}]
#     #                         )

#         # Robot State Publisher Node
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         parameters=[{
#             'robot_description': ParameterValue(
#                 Command(['xacro', 
#                          os.path.join( my_pkg_dir, 'urdf','mr_robot.xacro') ,
#                          'kinect_enabled:=', "true",
#                          ]),

#                 value_type=str
#             )
#         }],
#         remappings=[
#             ('/tf', 'tf'),
#             ('/tf_static', 'tf_static')
#         ]
#     )

#     # Joint State Publisher Node
#     joint_state_publisher_node = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher'
#     )
#     # # ROS-Gazebo Bridge Node
#     # ros_gz_bridge = Node(
#     #     package="ros_gz_bridge",
#     #     executable="parameter_bridge",
#     #     parameters=[{'config_file': bridge_config}]
#     # )

#     #spawn mr_robot using the topic "/mr_robot_description"
#     spawn_robot=Node(
#         package="ros_gz_sim",
#         executable='create',
#         name='spawn_mr_robot',
#         output='screen',
#         arguments=[
#                     '-entity', 'mr_robot',
#                     "-allow_renaming", "true",
#                     '-topic', '/robot_description',
#                     '-x', '-6.0',  # Set the initial X position
#                     '-y', '6.0',  # Set the initial Y position
#                     '-z', '0.0' ,  # Set the initial Z position
#                     '-Y', '0.0'   # Set the initial Z position
#     ],        
#     # output='screen',
#     #     parameters=[
#     #         {'use_sim_time': True},
#     #     ]
#     )


#     # ROS-Gazebo Bridge Node
#     ros_gz_bridge = Node(
#         package="ros_gz_bridge",
#         executable="parameter_bridge",
#         parameters=[{'config_file': bridge_config}]
#     )

#     return LaunchDescription([
#         # declare_world_arg,
#         gz_sim,
#        arg_use_sim_time,
#         robot_state_publisher_node,
#         joint_state_publisher_node,
#         spawn_robot,
#         ros_gz_bridge,
#         world,
#     ])






#! /usr/bin/env python3
import xacro
import os
from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
import launch.conditions
from launch_ros.descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable,DeclareLaunchArgument
def generate_launch_description():
    
    #path to xacro file
    xacro_file=get_package_share_directory('person_follower_sim')+'/urdf/mr_robot.xacro'
    my_pkg_dir = join(get_package_share_directory('person_follower_sim'), 'worlds', 'model.sdf')
    actor_world = join(get_package_share_directory("person_follower_sim"),'worlds','stand.world')
    actor_world1 = join(get_package_share_directory("person_follower_sim"),'worlds','walk.world')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    bridge_config=get_package_share_directory('person_follower_sim')+ '/config/bridge.yaml'
    rviz_config=get_package_share_directory("person_follower_sim")+"/config/rviz_config.rviz"

    # # Include the gazebo.launch.py file
    # gazebo=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
    #     launch_arguments={'pause': 'true',
    #                       'world':actor_world1}.items()
    # )


    SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory('person_follower_sim'))
    ),

    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args' :  actor_world1+ " -v 4"
            }.items()  
            
        )

    #spawn world
    world=Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        output='screen',
        arguments=['-entity', 'first_world', '-file', my_pkg_dir],
    )

    #publishing robot_state into topic robot_description
    robot_state=Node(package = 'robot_state_publisher',
                            executable = 'robot_state_publisher',
                            name='robot_state_publisher',
                            parameters = [{'robot_description': ParameterValue(Command( \
                                        ['xacro ', xacro_file,
                                        ' kinect_enabled:=', "true",
                                        # ' lidar_enabled:=', lidar_enabled,
                                        # ' camera_enabled:=', camera_enabled,
                                        ]), value_type=str)}]
                            )
    
    #spawn mr_robot using the topic "/mr_robot_description"
    robot_spawn=Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
                    '-entity', 'mr_robot',
                    '-topic', '/robot_description',
                    '-x', '-6.0',  # Set the initial X position
                    '-y', '6.0',  # Set the initial Y position
                    '-z', '0.0' ,  # Set the initial Z position
                    '-Y', '0.0'   # Set the initial Z position
    ]

    )

    #   # parameter bridge node to bridge different gz and tos 2 topics
    # ros_gz_bridge = Node(package="ros_gz_bridge", 
    #             executable="parameter_bridge",
    #             parameters = [
    #                 {'config_file': bridge_config}],
    #             # condition=IfCondition(with_bridge)
    #             )
    
    # # launch rviz node if rviz parameter was set to true
    # rviz = Node(package='rviz2',
    #             executable='rviz2',
    #             name='rviz',
    # #				output='screen',
    #             arguments=['-d' + rviz_config],
    #             # condition=IfCondition(with_rviz)
    #             )
    
    # map_stf = Node(package="tf2_ros",
    #                executable="static_transform_publisher",
    #                arguments=["0","0","0","0.0","0.0","0.0","map","odom"])
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time',default_value='true',
											description="Enable sim time from /clock")
    
                                             
    return LaunchDescription([
        gz_sim,
        world,
        robot_spawn,
        robot_state,
        # ros_gz_bridge,
        # rviz,
        arg_use_sim_time,
    ])