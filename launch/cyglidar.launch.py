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
        default_value = TextSubstitution(text="0"), 
        description = "pulse mode [0, 1] "
    )  ### 0: Auto, 1: Manual

    pulse_duration_arg = DeclareLaunchArgument(
        "pulse_duration", 
        default_value = TextSubstitution(text="10000"), 
        description = "pulse duration [0 to 10000] "
    ) 

    ld = LaunchDescription()

    lidar_node = launch_ros.actions.Node(            
        package = 'cyglidar_d1', executable = 'cyglidar_pcl_publisher', 
        output = 'screen',
        parameters=[
           {"port": "/dev/cyglidar"},
           {"baud_rate": 3000000},
           {"frame_id": "laser_link"},
           {"fixed_frame": "/map"},
           {"run_mode": LaunchConfiguration("version")},
           {"frequency": LaunchConfiguration("frequency_level")},
           {"set_auto_duration": LaunchConfiguration("pulse_control")},
           {"duration": LaunchConfiguration("pulse_duration")}
        ]
    )
    
    tf_node = launch_ros.actions.Node(
        package = 'tf2_ros', executable = "static_transform_publisher", name="to_laserlink",
        arguments = ["0", "0", "0", "0", "0", "0", "1", "map", "laser_link"]
    )

    ld.add_action(version_arg)
    ld.add_action(frequency_level_arg)
    ld.add_action(pulse_control_arg)
    ld.add_action(pulse_duration_arg)
    ld.add_action(lidar_node)
    ld.add_action(tf_node)

    return ld



'''
  <!-- Arguments -->
  <arg name="version" default="2" description="version type [0, 1, 2]"/> <!-- 0: 2D, 1: 3D, 2: Dual -->
  <arg name="frequency_level" default="0" description="level [0 to 15]"/>
  <arg name="pulse_control" default="0" description="pulse mode [0, 1]"/> <!-- 0: Auto, 1: Manual -->
  <arg name="pulse_duration" default="10000" description="duration [0 to 10000]"/>

  <node pkg="cyglidar_d1" exec="cyglidar_pcl_publisher" name="line_laser" output="screen">
    <param name="port" value="/dev/cyglidar"/>
    <param name="baud_rate" value="3000000"/>
    <param name="frame_id" value="laser_link"/>
    <param name="fixed_frame" value="/map"/>
    <param name="run_mode" value="$(arg version)"/>
    <param name="frequency" value="$(arg frequency_level)"/>
    <param name="set_auto_duration" value="$(arg pulse_control)"/>
    <param name="duration" value="$(arg pulse_duration)"/>
  </node>

  <node pkg="rviz" exec="rviz2" name="rvizs" args="-d $(find cyglidar_d1)/rviz/cyglidar_config.rviz"/>
  <node pkg="tf" exec="static_transform_publisher" name="to_laserlink" args="0 0 0 0 0 0 1 map laser_link 10"/>
</launch>
'''