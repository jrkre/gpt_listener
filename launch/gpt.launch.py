from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# see this link for any help with creating launch files: https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

def generate_launch_description():

	### ARGUMENTS
    main_group = GroupAction(
                        actions=[
                                Node( package='gpt_listener', executable='gpt_node', output='log', emulate_tty=True ),
                                Node( package='gpt_listener', executable='gpt_talker', output='log', emulate_tty=True ),                               
                                ]
    )
    include1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('voskros'),
                'launch',
                'voskros.launch.yaml'
            ])
        ])
    )   
    
    return LaunchDescription([ main_group, include1 ])