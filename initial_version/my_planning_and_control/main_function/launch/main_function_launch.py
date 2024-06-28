import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="main_function",
            executable="main_function"
        )
    ])