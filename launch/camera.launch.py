from launch import LaunchDescription
from launch_ros.actions import Node  

def generate_launch_description():

    camera_node = Node(
        package='pi_camera',
        executable='camera_pub',
        name='pi_camera_pub_node',
        output='screen'  
    )

    return LaunchDescription([
        camera_node
    ])
