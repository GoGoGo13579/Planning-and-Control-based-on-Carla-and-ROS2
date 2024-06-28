import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="my_pnc",
            executable="planning_agent"
        ),
        launch_ros.actions.Node(
            package="my_pnc",
            executable="control_agent"
        )
    ])