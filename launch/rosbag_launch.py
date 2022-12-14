from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    
    
    freq_pub = LaunchConfiguration('freq_pub')
    record_all_topics = LaunchConfiguration('record_all_topics')

    return LaunchDescription([

        
        DeclareLaunchArgument(
            'freq_pub',
            default_value='4.0'
        ),

        DeclareLaunchArgument(
            'record_all_topics',
            default_value='True'
        ),

        Node(
            package='beginner_tutorials',
            executable='talker',
            name='minimal_publisher',
            parameters=[{
                "freq_pub": LaunchConfiguration('freq_pub'),
            }]
        ),

        Node(
            package='beginner_tutorials',
            executable='listener',
            name='minimal_subscriber',
            
        ),


        ExecuteProcess(
        condition=IfCondition(record_all_topics),
        cmd=[
            'ros2', 'bag', 'record', '-o all_topics_bag', '-a'
        ],
        shell=True
        )

    ])