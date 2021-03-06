import launch
import launch_ros.actions

def generate_launch_description():
    yolov5_ros = launch_ros.actions.Node(
        package="yolov5_ros", executable="yolov5_ros",
        parameters=[
            {"view_img": True},
            {"device": "0"},
        ],
    )
    return launch.LaunchDescription([
        yolov5_ros,
    ])