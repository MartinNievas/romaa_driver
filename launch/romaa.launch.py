from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # Node action
    drv_node = Node(
        package='romaa_driver',
        executable='romaa_driver',
        name='romaa_driver'
    )

    # Adding actions
    ld.add_action(drv_node)
    return ld
