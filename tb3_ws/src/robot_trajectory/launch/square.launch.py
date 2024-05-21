from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='robot_trajectory',
      executable='square_odom',
      # remappings=[
      #   ('/cmd_vel', '/turtle1/cmd_vel'),           # ros2 run robot_trajectory square_odom --ros-args --remap /cmd_vel:=/turtle1/cmd_vel
      # ],
      parameters=[
        {'linear_speed': 0.6},
        {'angular_speed': 0.8},
        {'square_length': 0.3}
      ]
    )
  ])
