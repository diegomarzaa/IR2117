from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
  return LaunchDescription([

  Node(
    package='obstacles',
    executable='detector',
    namespace='southwest',
    parameters=[
      {"obs_angle_min": 1.5708},
      {"obs_angle_max": 1.9635},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),

  Node(
    package='obstacles',
    executable='detector',
    namespace='west',
    parameters=[
      {"obs_angle_min": 1.1781},
      {"obs_angle_max": 1.5708},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),


  Node(
    package='obstacles',
    executable='detector',
    namespace='northwest',
    parameters=[
      {"obs_angle_min": 0.3927},
      {"obs_angle_max": 1.1781},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),


  Node(
    package='obstacles',
    executable='detector',
    namespace='north',
    parameters=[
      {"obs_angle_min": -0.3927},
      {"obs_angle_max": 0.3927},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),
  
  
  Node(
    package='obstacles',
    executable='detector',
    namespace='northeast',
    parameters=[
      {"obs_angle_min": -1.1781},
      {"obs_angle_max": -0.3927},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),

Node(
    package='obstacles',
    executable='detector',
    namespace='east',
    parameters=[
      {"obs_angle_min": -1.1781},
      {"obs_angle_max": -1.5708},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),

      
  Node(
    package='obstacles',
    executable='detector',
    namespace='southeast',
    parameters=[
      {"obs_angle_min": -1.5708},
      {"obs_angle_max": -1.9635},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),

  Node(
    package='obstacles',
    executable='detector',
    namespace='south',
    parameters=[
      {"obs_angle_min": -1.9635},
      {"obs_angle_max": 1.9635},
      {"obs_threshold": 2.0}],
    remappings=[
      ('scan', '/scan')
    ]
    ),
  

  Node(
    package='intrusion',
    executable='monitor',
  )
])