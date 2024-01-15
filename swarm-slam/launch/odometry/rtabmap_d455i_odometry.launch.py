from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    parameters=[{
          'frame_id': LaunchConfiguration('namespace').perform(context)[1:] + '_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False, # Set to True for OAK-D
          'wait_imu_to_init':True,
          }]

    remappings=[
          ('imu', LaunchConfiguration('namespace').perform(context) + '/forward/imu/data'),
          ('rgb/image', LaunchConfiguration('namespace').perform(context) + '/color/image_raw'),
          ('rgb/camera_info', LaunchConfiguration('namespace').perform(context) + '/color/camera_info'),
          ('depth/image', LaunchConfiguration('namespace').perform(context) + '/aligned_depth_to_color/image_raw'),
          ('odom', LaunchConfiguration('namespace').perform(context) + '/odom'),]

    return [
        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen', name='rgbd_odometry',
            parameters=parameters,
            remappings=remappings,
            ),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', LaunchConfiguration('namespace').perform(context) + '/forward/imu'),
                        ('imu/data', LaunchConfiguration('namespace').perform(context) + '/forward/imu/data')]),
                # The IMU frame is missing in TF tree, add it:

        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ]

def generate_launch_description():
    
    return LaunchDescription([            
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        OpaqueFunction(function=launch_setup)
    ])
