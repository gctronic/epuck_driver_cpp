from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, FileContent
import launch
from launch.conditions import IfCondition
import launch_ros.actions


def generate_launch_description():
    launch_args = list(
        map(
            lambda x: launch.actions.DeclareLaunchArgument(
                name=x[0],
                default_value=x[1],
            ),
            {
                "namespace": "/",
                "epuck2_id": "0",
                "epuck2_address": "192.168.1.1",
                "epuck2_port": "1000",
                "epuck2_name": "epuck2_robot_0",
                "rviz": "true",
                "xpos": "0.0",
                "ypos": "0.0",
                "theta": "0.0",
                "cam_en": "false",
                "floor_en": "false",
                "sim_en": "false",
            }.items(),
        )
    )

    ld = launch.LaunchDescription(
        launch_args
        + [
            launch_ros.actions.Node(
                condition=IfCondition(
                    launch.substitutions.LaunchConfiguration("rviz")
                ),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epuck_driver_cpp"),
                            "config",
                            "single_epuck2_driver_rviz.rviz",
                        ]
                    ),
                ],
            ),
            # Optionally launch a simple server to simulate the connection to the e-puck2 robot
            launch.actions.execute_process.ExecuteProcess(
                name="epuck2_sim",
                condition=IfCondition(
                    launch.substitutions.LaunchConfiguration("sim_en")
                ),
                output="screen",
                cmd=[
                    "python3",
                    [
                        FindPackageShare("epuck_driver_cpp"),
                        "/scripts/epuck2_sim",
                    ],
                    "--ip",
                    launch.substitutions.LaunchConfiguration("epuck2_address"),
                    "--port",
                    launch.substitutions.LaunchConfiguration("epuck2_port"),
                ],
            ),
            launch_ros.actions.Node(
                package="epuck_driver_cpp",
                executable="epuck_driver_cpp",
                name="epuck_driver_cpp",
                namespace=launch.substitutions.LaunchConfiguration("namespace"),
                output="screen",
                prefix=[
                    # Debugging with gdb
                    # "xterm -bg black -fg white -fa 'Monospace' -fs 13 -e gdb -ex start --args"
                ],
                parameters=[
                    {
                        "epuck2_id": launch.substitutions.LaunchConfiguration(
                            "epuck2_id"
                        )
                    },
                    {
                        "epuck2_address": launch.substitutions.LaunchConfiguration(
                            "epuck2_address"
                        )
                    },
                    {
                        "epuck2_port": launch.substitutions.LaunchConfiguration(
                            "epuck2_port"
                        )
                    },
                    {
                        "epuck2_name": launch.substitutions.LaunchConfiguration(
                            "epuck2_name"
                        )
                    },
                    {"robot/xpos": launch.substitutions.LaunchConfiguration("xpos")},
                    {"robot/ypos": launch.substitutions.LaunchConfiguration("ypos")},
                    {"robot/theta": launch.substitutions.LaunchConfiguration("theta")},
                    {
                        "robot/camera": launch.substitutions.LaunchConfiguration(
                            "cam_en"
                        )
                    },
                    {
                        "robot/floor": launch.substitutions.LaunchConfiguration(
                            "floor_en"
                        )
                    },
                ],
            ),
            launch_ros.actions.Node(
                namespace=launch.substitutions.LaunchConfiguration("namespace"),
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name=(
                    "epuck_state_publisher_",
                    launch.substitutions.LaunchConfiguration("epuck2_id"),
                ),
                parameters=[
                    {"use_sim_time": True},
                    {
                        "frame_prefix": [
                            launch.substitutions.LaunchConfiguration("epuck2_name"),
                            "/",
                        ],
                        "publish_frequency": 60.0,
                        "robot_description": FileContent(
                            [
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("epuck_driver_cpp"),
                                        "urdf",
                                        "epuck_urdf.xml",
                                    ]
                                ),
                            ]
                        ),
                    },
                ],
            ),
            launch_ros.actions.Node(
                namespace=launch.substitutions.LaunchConfiguration("namespace"),
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name=(
                    "epuck_joint_publisher_",
                    launch.substitutions.LaunchConfiguration("epuck2_id"),
                ),
                arguments=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("epuck_driver_cpp"),
                            "urdf",
                            "epuck_urdf.xml",
                        ]
                    ),
                ],
            ),
            # launch_ros.actions.Node(
            #     package="gmapping",
            #     executable="slam_gmapping",
            #     name="slam_gmapping",
            #     output="screen",
            #     parameters=[
            #         {
            #             "base_frame": [
            #                 launch.substitutions.LaunchConfiguration("epuck2_name"),
            #                 launch.substitutions.TextSubstitution(
            #                     text=' + "/base_link"'
            #                 ),
            #             ],
            #         },
            #         {"map_update_interval": "1.0"},
            #         {"linearUpdate": "0.01"},
            #         {"angularUpdate": "0.09"},
            #         {"temporalUpdate": "-1.0"},
            #         {"xmin": "-1.0"},
            #         {"xmax": "1.0"},
            #         {"ymin": "-1.0"},
            #         {"ymax": "1.0"},
            #         {"delta": "0.005"},
            #         {"maxRange": "0.080"},
            #         {"maxUrange": "0.075"},
            #         {"sigma": "0.05"},
            #         {"kernelSize": "1"},
            #         {"lstep": "0.01"},
            #         {"astep": "0.02"},
            #         {"iterations": "5"},
            #         {"lsigma": "0.075"},
            #         {"ogain": "3.0"},
            #         {"lskip": "0"},
            #         {"minimumScore": "100.0"},
            #         {"srr": "0.01"},
            #         {"srt": "0.02"},
            #         {"str": "0.01"},
            #         {"stt": "0.02"},
            #         {"resampleThreshold": "0.5"},
            #         {"particles": "30"},
            #         {"llsamplerange": "0.01"},
            #         {"llsamplestep": "0.01"},
            #         {"lasamplerange": "0.005"},
            #         {"lasamplestep": "0.005"},
            #         {"occ_thresh": "0.25"},
            #         {"transform_publish_period": "0.05"},
            #     ],
            # ),
        ]
    )
    return ld


if __name__ == "__main__":
    generate_launch_description()
