
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
    pkg_follower_desc = get_package_share_directory('person_follower_sim')
    my_pkg_dir = join(get_package_share_directory('person_follower_sim'), 'worlds', 'model.sdf')
    actor_world = join(get_package_share_directory("person_follower_sim"),'worlds','stand.world')
    actor_world1 = join(get_package_share_directory("person_follower_sim"),'worlds','walk.world')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # bridge_config = os.path.join(pkg_follower_desc, 'config', 'bridge.yaml')
    rviz_config_file = os.path.join(pkg_follower_desc, 'config', 'display.rviz')
    # # Include the gazebo.launch.py file
    # gazebo=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
    #     launch_arguments={'pause': 'true',
    #                       'world':actor_world1}.items()
    # )
      # parameter bridge node to bridge different gz and tos 2 topics
    # ros_gz_bridge = Node(package="ros_gz_bridge", 
    #             executable="parameter_bridge",
    #             parameters = [
    #                 {'config_file': bridge_config}],
    #             # condition=IfCondition(with_bridge)
    #             )
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time',default_value='true',
											description="Enable sim time from /clock")
    

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
                                        # ' kinect_enabled:=', "true",
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
                    "-allow_renaming", "true",
                    '-x', '-6.0',  # Set the initial X position
                    '-y', '6.0',  # Set the initial Y position
                    '-z', '0.0' ,  # Set the initial Z position
                    '-Y', '0.0',  # Set the initial Z position
 ],
     
    
    )

    
   # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
           '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock ',
            '/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # map_stf = Node(package="tf2_ros",
    #                executable="static_transform_publisher",
    #                arguments=["0","0","0","0.0","0.0","0.0","map","odom"])
   
                                             
    return LaunchDescription([
        arg_use_sim_time,
        robot_spawn,
        robot_state,
        gz_sim,
        world,
        gz_bridge_node,
        rviz_node,
        
    ])