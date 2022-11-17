
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='publisher'
        ),
        Node(
            package='beginner_tutorials',
            executable='server',
            name='response_server'
        ),
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener',
        )
    ])