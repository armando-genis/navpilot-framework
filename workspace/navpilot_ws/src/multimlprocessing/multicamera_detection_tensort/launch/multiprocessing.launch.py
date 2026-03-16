import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('multicamera_detection_tensort')
    paramsConfig = os.path.join(pkg_share, 'config', 'params.yaml')
    additional_env = {'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}'}

    multicamera_detection_tensort_node = launch_ros.actions.Node(
        package='multicamera_detection_tensort',
        executable='multicamera_detection_tensort_node',
        name='multicamera_detection_tensort_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env=additional_env,
    )
    
    return launch.LaunchDescription([
        multicamera_detection_tensort_node
    ])