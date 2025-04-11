import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    生成ROS 2启动描述，用于启动Navigation2和RViz2。

    此函数设置必要的路径和参数，然后启动Navigation2的bringup_launch.py
    并启动RViz2以可视化导航过程。

    返回:
        launch.LaunchDescription: 包含启动配置的LaunchDescription对象。
    """
    # 获取与拼接默认路径
    # 获取fishbot_navigation2包的共享目录路径
    fishbot_navigation2_dir = get_package_share_directory(
        'fishbot_navigation2')
    # 获取nav2_bringup包的共享目录路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # 拼接RViz配置文件的完整路径
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    # 定义一个名为use_sim_time的启动配置参数，默认值为true
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    # 定义一个名为map的启动配置参数，默认值为fishbot_navigation2包中maps目录下的room.yaml文件路径
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    # 定义一个名为params_file的启动配置参数，默认值为fishbot_navigation2包中config目录下的nav2_params.yaml文件路径
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        # 声明use_sim_time启动参数，设置默认值和描述信息
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        # 声明map启动参数，设置默认值和描述信息
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        # 声明params_file启动参数，设置默认值和描述信息
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        # 包含Navigation2的bringup_launch.py启动文件
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        # 启动RViz2节点
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])