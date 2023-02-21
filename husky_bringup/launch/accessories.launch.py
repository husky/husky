import os
import yaml

import ament_index_python.packages
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    # Primary Lidar Environment Variables
    primary_lidar_enable = EnvironmentVariable('CPR_LASER', default_value='false')
    primary_lidar_model = EnvironmentVariable('CPR_LASER_MODEL', default_value='ust10')
    primary_lidar_ip = EnvironmentVariable('CPR_LASER_HOST', default_value='192.168.131.20')
    primary_lidar_topic = EnvironmentVariable('CPR_LASER_TOPIC', default_value='front/scan')
    primary_lidar_mount = EnvironmentVariable('CPR_LASER_MOUNT', default_value='front_laser')

    if (primary_lidar_enable.perform(lc)) == 'true':
        if (primary_lidar_model.perform(lc) == 'ust10'):
            node_hokuyo = Node(
                package='urg_node', 
                executable='urg_node_driver', 
                name='urg_node',
                output='screen',
                remappings={('scan', primary_lidar_topic)},
                parameters=[
                    {'ip_address': primary_lidar_ip},
                    {'laser_frame_id': primary_lidar_mount}
                ]
            )
            ld.add_action(node_hokuyo)


    # Secondary Lidar Environment Variables
    secondary_lidar_enable = EnvironmentVariable('CPR_LASER_SECONDARY', default_value='false')
    secondary_lidar_model = EnvironmentVariable('CPR_LASER_SECONDARY_MODEL', default_value='ust10')
    secondary_lidar_ip = EnvironmentVariable('CPR_LASER_SECONDARY_HOST', default_value='192.168.131.21')
    secondary_lidar_topic = EnvironmentVariable('CPR_LASER_SECONDARY_TOPIC', default_value='rear/scan')
    secondary_lidar_mount = EnvironmentVariable('CPR_LASER_SECONDARY_MOUNT', default_value='rear_laser')

    if (secondary_lidar_enable.perform(lc)) == 'true':
        if (secondary_lidar_model.perform(lc) == 'ust10'):
            node_hokuyo2 = Node(
                package='urg_node',
                executable='urg_node_driver',
                name='urg_node',
                output='screen',
                remappings={('scan', secondary_lidar_topic)},
                parameters=[
                    {'ip_address': secondary_lidar_ip},
                    {'laser_frame_id': secondary_lidar_mount}
                ]
            )
            ld.add_action(node_hokuyo2)

    # Primary 3D Lidar Environment Variables
    primary_lidar_3d_enable = EnvironmentVariable('CPR_3D_LASER', default_value='false')
    primary_lidar_3d_model = EnvironmentVariable('CPR_3D_LASER_MODEL', default_value='vlp16')
    primary_lidar_3d_ip = EnvironmentVariable('CPR_3D_LASER_HOST', default_value='192.168.131.20')
    primary_lidar_3d_topic = EnvironmentVariable('CPR_3D_LASER_TOPIC', default_value='mid/points')
    primary_lidar_3d_mount = EnvironmentVariable('CPR_3D_LASER_MOUNT', default_value='mid_velodyne')
    
    if (primary_lidar_3d_enable.perform(lc)) == 'true':
        if (primary_lidar_3d_model.perform(lc) == 'vlp16'):
                
            velodyne_pointcloud_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')

            velodyne_pointcloud_params_file = os.path.join(velodyne_pointcloud_dir, 'config', 'VLP16-velodyne_convert_node-params.yaml')

            with open(velodyne_pointcloud_params_file, 'r') as f:
                config_velodyne_pointcloud_vlp16_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
            config_velodyne_pointcloud_vlp16_params['calibration'] = os.path.join(velodyne_pointcloud_dir, 'params', 'VLP16db.yaml')

            node_velodyne_driver = Node(
                package='velodyne_driver', 
                executable='velodyne_driver_node', 
                name='velodyne_driver_node',
                output='screen',
                remappings={('scan', primary_lidar_3d_topic)},
                parameters=[
                    {'device_ip': primary_lidar_3d_ip},
                    {'frame_id': primary_lidar_3d_mount},
                    {'gps_time': False},
                    {'time_offset': 0.0},
                    {'enabled': True},
                    {'read_once': False},
                    {'read_fast': False},
                    {'repeat_delay': 0.0},
                    {'model': 'VLP16'},
                    {'rpm': 600.0},
                    {'port': 2368}
                ]
            )

            node_velodyne_convert = Node(
                package='velodyne_pointcloud',
                executable='velodyne_convert_node',
                output='screen',
                remappings={('velodyne_points', primary_lidar_3d_topic)},
                parameters=[config_velodyne_pointcloud_vlp16_params])

            ld.add_action(node_velodyne_driver)
            ld.add_action(node_velodyne_convert)
        
        elif (primary_lidar_3d_model.perform(lc) == 'vlp32c'):

            velodyne_pointcloud_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')

            velodyne_pointcloud_params_file = os.path.join(velodyne_pointcloud_dir, 'config', 'VLP32C-velodyne_convert_node-params.yaml')

            with open(velodyne_pointcloud_params_file, 'r') as f:
                config_velodyne_pointcloud_vlp32c_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
            config_velodyne_pointcloud_vlp32c_params['calibration'] = os.path.join(velodyne_pointcloud_dir, 'params', 'VeloView-VLP-32C.yaml')

            node_velodyne_driver = Node(
                package='velodyne_driver', 
                executable='velodyne_driver_node', 
                name='velodyne_driver_node',
                output='screen',
                remappings={('scan', primary_lidar_3d_topic)},
                parameters=[
                    {'device_ip': primary_lidar_3d_ip},
                    {'frame_id': primary_lidar_3d_mount},
                    {'gps_time': False},
                    {'time_offset': 0.0},
                    {'enabled': True},
                    {'read_once': False},
                    {'read_fast': False},
                    {'repeat_delay': 0.0},
                    {'model': '32C'},
                    {'rpm': 600.0},
                    {'port': 2368}
                ]
            )

            node_velodyne_convert = Node(
                package='velodyne_pointcloud',
                executable='velodyne_convert_node',
                output='screen',
                remappings={('velodyne_points', primary_lidar_3d_topic)},
                parameters=[config_velodyne_pointcloud_vlp16_param])

            ld.add_action(node_velodyne_driver)
            ld.add_action(node_velodyne_convert)


    # This is disabled since nmea_navsat_driver has not been released.
    # Primary GPS Environment Variables
    # primary_gps_enable = EnvironmentVariable('CPR_GPS', default_value='false')
    # primary_gps_port = EnvironmentVariable('CPR_GPS_PORT', default_value='/dev/clearpath/gps')
    # primary_gps_baud = EnvironmentVariable('CPR_GPS_BAUD', default_value='57600')
    # primary_gps_mount = EnvironmentVariable('CPR_GPS_MOUNT', default_value='gps_link')

    # if (primary_gps_enable.perform(lc)) == 'true':
    #     node_gps = Node(
    #         package='nmea_navsat_driver',
    #         executable='nmea_serial_driver',
    #         name='nmea_serial_driver',
    #         output='screen',
    #         parameters=[
    #             {'port': primary_gps_port},
    #             {'baud': primary_gps_baud},
    #             {'frame_id': primary_gps_mount}
    #         ]
    #     )
    #     ld.add_action(node_gps)


    # Primary IMU Environment Variables
    primary_imu_enable = EnvironmentVariable('CPR_IMU', default_value='false')
    primary_imu_model = EnvironmentVariable('CPR_IMU_MODEL', default_value='microstrain')
    primary_imu_port = EnvironmentVariable('CPR_IMU_PORT', default_value='/dev/microstrain')
    primary_imu_baud = EnvironmentVariable('CPR_IMU_BAUD', default_value='115200')
    primary_imu_mount = EnvironmentVariable('CPR_IMU_MOUNT', default_value='imu_link')

    if (primary_imu_enable.perform(lc)) == 'true':
        if (primary_imu_model.perform(lc) == 'microstrain'):
            launch_microstrain = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("microstrain_inertial_driver"), 'launch', 'microstrain_launch.py']
                    )
                ),
                launch_arguments={
                    'port': primary_imu_port,
                    'baudrate': primary_imu_baud,
                    'imu_frame_id': primary_imu_mount,
                    'configure' : 'true',
                    'activate' : 'true',
                    'use_enu_frame' : 'true'
                }.items()
            )
            ld.add_action(launch_microstrain)

    # Primary Realsense Environment Variables
    primary_realsense_enable = EnvironmentVariable('CPR_REALSENSE', default_value='false')

    if (primary_realsense_enable.perform(lc)) == 'true':
        launch_primary_realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), 'launch', 'rs_launch.py']
                )
            ),
            launch_arguments={
                'pointcloud.enable' : 'true'
            }.items()
        )
        ld.add_action(launch_primary_realsense)


    return ld

