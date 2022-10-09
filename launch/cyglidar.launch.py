from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
import launch_ros.actions

def generate_launch_description():    
    
    version_arg = DeclareLaunchArgument(
        "version", 
        default_value = TextSubstitution(text="2"), 
        description = "version type [0, 1, 2]"
    )  ### 0: 2D, 1: 3D, 2: Dual

    frequency_level_arg = DeclareLaunchArgument(
        "frequency_level", 
        default_value = TextSubstitution(text="0"), 
        description = "level [0 to 15]"
    )

    pulse_control_arg = DeclareLaunchArgument(
        "pulse_control", 
        default_value = TextSubstitution(text="1"), 
        description = "pulse mode [0, 1] "
    )  ### 0: Auto, 1: Manual

    pulse_duration_arg = DeclareLaunchArgument(
        "pulse_duration", 
        default_value = TextSubstitution(text="2700"), 
        description = "pulse duration [0 to 10000] "
    ) 

    ld = LaunchDescription()

    lidar_node = launch_ros.actions.Node(            
        package = 'cyglidar_d1', executable = 'cyglidar_pcl_publisher', 
        output = 'screen',
        parameters=[
           {"port": "/dev/cyglidar"},
           {"baud_rate": 3000000},
           {"frame_id": "cyglidar_link"},
           {"fixed_frame": "/base_link"},
           {"run_mode": LaunchConfiguration("version")},
           {"frequency": LaunchConfiguration("frequency_level")},
           {"set_auto_duration": LaunchConfiguration("pulse_control")},
           {"duration": LaunchConfiguration("pulse_duration")}
        ]
    )
    
    tf_node = launch_ros.actions.Node(
        package = 'tf2_ros', executable = "static_transform_publisher", name="cyglidar_to_base_link",
        arguments = ["0.2162", "0.00", "0.259675", "0", "0.226893", "0", "base_link", "cyglidar_link"]
    )

    ld.add_action(version_arg)
    ld.add_action(frequency_level_arg)
    ld.add_action(pulse_control_arg)
    ld.add_action(pulse_duration_arg)
    ld.add_action(lidar_node)
    ld.add_action(tf_node)

    return ld




