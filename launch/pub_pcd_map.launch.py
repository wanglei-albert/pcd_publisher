import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pcd_file_path = LaunchConfiguration('pcd_file',
                                        default=os.path.join(get_package_share_directory('go2_nav'),
                                                             'maps',
                                                             'map.pcd'))

    ld = LaunchDescription()

    pub_pcd_node = Node(
        package="pcd_publisher",
        executable="pcd_publisher",
        output='screen',
        emulate_tty=True,
        parameters=[{'pcd_file':pcd_file_path}],
    )

    ld.add_action(pub_pcd_node)
    return ld