import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)


def generate_launch_description():
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "robot_id0": "3000",
                "robot_id1": "0",  # 0 means not used
                "robot_id2": "0",  # 0 means not used
                "robot_id3": "0",  # 0 means not used
                "robot_addr0": "192.168.1.1",
                "robot_addr1": "192.168.1.2",
                "robot_addr2": "192.168.1.3",
                "robot_addr3": "192.168.1.4",
                "robot_port0": "1000",
                "robot_port1": "1000",
                "robot_port2": "1000",
                "robot_port3": "1000",
                "robot_sim_en0": "false",
                "robot_sim_en1": "false",
                "robot_sim_en2": "false",
                "robot_sim_en3": "false",
                "rviz": "true",
            }.items(),
        )
    )

    robot_poses = [
        ("-0.1", "-0.1", "0.0"),
        ("0.1", "-0.1", "0.0"),
        ("-0.1", "0.1", "0.0"),
        ("0.1", "0.1", "0.0"),
    ]

    launch_robots = []
    for i in range(4):
        launch_robots.append(
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epuck_driver_cpp"),
                            "launch",
                            "epuck2_controller.launch.py",
                        ]
                    )
                ),
                condition=launch.conditions.IfCondition(
                    PythonExpression(
                        [
                            f"{i} == 0 or (",
                            launch.substitutions.LaunchConfiguration(
                                f"robot_id{i}"),
                            " != 0)",
                        ]
                    )
                ),
                launch_arguments={
                    "namespace": f"epuck2_robot_{i}",
                    "epuck2_id": launch.substitutions.LaunchConfiguration(
                        f"robot_id{i}"
                    ),
                    "epuck2_address": launch.substitutions.LaunchConfiguration(
                        f"robot_addr{i}"
                    ),
                    "epuck2_port": launch.substitutions.LaunchConfiguration(
                        f"robot_port{i}"
                    ),
                    "epuck2_name": f"epuck2_robot_{i}",
                    "cam_en": "false",
                    "floor_en": "false",
                    "xpos": robot_poses[i][0],
                    "ypos": robot_poses[i][1],
                    "theta": robot_poses[i][2],
                    "rviz": "false",
                    "sim_en": launch.substitutions.LaunchConfiguration(
                        f"robot_sim_en{i}"
                    ),
                }.items(),
            )
        )

    ld = launch.LaunchDescription(
        launch_args
        + launch_robots
        + [
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration("rviz")
                ),
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epuck_driver_cpp"),
                            "config",
                            "multi_epuck2_driver_rviz.rviz",
                        ]
                    ),
                ],
            ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
