import launch
import launch.launch_description_sources
import launch_ros.actions
import os
import sys
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kp_lateral',
            default_value='0.7'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Ki_lateral',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kd_lateral',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kp_longitudinal',
            default_value='0.206'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Ki_longitudinal',
            default_value='0.0206'
        ),
        launch.actions.DeclareLaunchArgument(
            name='Kd_longitudinal',
            default_value='0.515'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_time_step',
            default_value='0.05'
        ),
        launch_ros.actions.Node(
            package='carla_ad_agent',
            executable='ad_agent',
            name=['carla_ad_agent_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                },
                {
                    'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk')
                }
            ]
        ),
        # launch_ros.actions.Node(
        #     package='carla_ad_agent',
        #     executable='local_planner',
        #     name=['local_planner_', launch.substitutions.LaunchConfiguration('role_name')],
        #     output='screen',
        #     parameters=[
        #         {
        #             'use_sim_time': True
        #         },
        #         {
        #             'role_name': launch.substitutions.LaunchConfiguration('role_name')
        #         },
        #         {
        #             'Kp_lateral': launch.substitutions.LaunchConfiguration('Kp_lateral')
        #         },
        #         {
        #             'Ki_lateral': launch.substitutions.LaunchConfiguration('Ki_lateral')
        #         },
        #         {
        #             'Kd_lateral': launch.substitutions.LaunchConfiguration('Kd_lateral')
        #         },
        #         {
        #             'Kp_longitudinal': launch.substitutions.LaunchConfiguration('Kp_longitudinal')
        #         },
        #         {
        #             'Ki_longitudinal': launch.substitutions.LaunchConfiguration('Ki_longitudinal')
        #         },
        #         {
        #             'Kd_longitudinal': launch.substitutions.LaunchConfiguration('Kd_longitudinal')
        #         },
        #         {
        #             'control_time_step': launch.substitutions.LaunchConfiguration('control_time_step')
        #         }
        #     ]
        # ),


        # launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource
        #     (
        #         os.path.join(get_package_share_directory('carla_ad_agent_cpp'),'launch/local_planner_cpp_launch.py')
        #     ),
        #     launch_arguments={
        #         'role_name': launch.substitutions.LaunchConfiguration('role_name'),
        #         'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk'),
        #         'Kp_lateral': launch.substitutions.LaunchConfiguration('Kp_lateral')
        #     }.items()
        # )

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("main_function"),'launch/main_function_launch.py')
            )
            
        )

    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
