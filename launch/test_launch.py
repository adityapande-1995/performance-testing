import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['./nodes/publisher.py'],
            shell=True
        ),
        launch.actions.ExecuteProcess(
            cmd=['./nodes/subscriber.py'],
            shell=True
        ),
    ])
