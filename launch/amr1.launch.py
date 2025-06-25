import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch_ros.actions import Node

def generate_launch_description():
    # this name has to match the root_name in the Xacro file
    robotXacroName = 'differential_drive_robot'
    # this is the name of our package
    namePackage = 'amr_1'
    # relative path to the xacro file
    modelFileRelativePath = 'model/amr1.xacro'
    # relative path to the Gazebo world file
    worldFileRelativePath = 'model/empty_world.world'
    # absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    # absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    # get the robot description from xacro
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # gazebo_ros package launch file
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'),
        'launch', 'gazebo.launch.py'))

    # launch description for gazebo
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, 
        launch_arguments={'world': pathWorldFile}.items())

    # spawn entity node
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )

    # robot state publisher node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                   'use_sim_time': True}]
    )
    
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', os.path.join(get_package_share_directory(namePackage), 'rviz', 'amr_config.rviz')],
parameters=[{'use_sim_time': True}]
)
    

    # create launch description object
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_node)
    # add gazebo launch
    launchDescriptionObject.add_action(gazeboLaunch)
    # add nodes
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject
