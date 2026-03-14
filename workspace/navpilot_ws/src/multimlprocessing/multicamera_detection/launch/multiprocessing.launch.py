import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # So the node can load libonnxruntime.so.1 (used by YOLOs-CPP)
    pkg_share = get_package_share_directory('multicamera_detection')
    # .../install/multicamera_detection/share/multicamera_detection -> 4 dirnames -> workspace root
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    onnx_lib_dir = os.path.join(
        workspace_root, 'src', 'multimlprocessing', 'YOLOs-CPP', 'onnxruntime-linux-x64-1.20.1', 'lib'
    )
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    if os.path.isdir(onnx_lib_dir):
        ld_library_path = onnx_lib_dir + ((':' + ld_library_path) if ld_library_path else '')
    additional_env = {
        'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}',
        'LD_LIBRARY_PATH': ld_library_path,
    }

    paramsConfig = os.path.join(pkg_share, 'config', 'params.yaml')

    multicamera_detection_node = launch_ros.actions.Node(
        package='multicamera_detection',
        executable='multicamera_detection_node',
        name='multicamera_detection_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env=additional_env,
    )
    
    return launch.LaunchDescription([
        multicamera_detection_node
    ])