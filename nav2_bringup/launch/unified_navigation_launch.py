#### localization + naivgation => 하나로 통합된 네비게이션 런치 파일 ####

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, 
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile, ComposableNode
from nav2_common.launch import RewrittenYaml, ReplaceString

## 강제 종료를 위한 모듈 ## 
from launch.actions import EmitEvent
from launch.events import Shutdown


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    graph_filepath = LaunchConfiguration('graph')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'autostart': autostart
        }

    # Only it applys when `use_namespace` is True.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='', description='Path to the graph file to load'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Lifecycle nodes for localization and navigation
    localization_lifecycle_nodes = ['map_server']
    
    navigation_lifecycle_nodes = ['controller_server',
                                 'smoother_server',
                                 'planner_server',
                                 'behavior_server',
                                 'bt_navigator',
                                 'waypoint_follower',
                                 'velocity_smoother',
                                #  'route_server',
                                 ]

    # Specify the actions
    unified_nav_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        # Container node for composition
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
            on_exit=[EmitEvent(event=Shutdown())]
            ),

        # Localization nodes (non-composition)
        GroupAction(
            condition=IfCondition(PythonExpression(['not ', use_composition, ' and not ', slam])),
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings),
                # Node(
                #     package='nav2_amcl',
                #     executable='amcl',
                #     name='amcl',
                #     output='screen',
                #     respawn=use_respawn,
                #     respawn_delay=2.0,
                #     parameters=[configured_params],
                #     arguments=['--ros-args', '--log-level', log_level],
                #     remappings=remappings),            
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': localization_lifecycle_nodes}],
                    on_exit=[EmitEvent(event=Shutdown())]
                    ),
            ]
        ),

        # Navigation nodes (non-composition)
        GroupAction(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings),
                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings),
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                # Node(
                #     package='nav2_route',
                #     executable='route_server',
                #     name='route_server',
                #     output='screen',
                #     respawn=use_respawn,
                #     respawn_delay=2.0,
                #     parameters=[configured_params, {'graph_filepath': graph_filepath}],
                #     arguments=['--ros-args', '--log-level', log_level],
                #     remappings=remappings),
                # Node(
                #     package='nav2_collision_monitor',
                #     executable='collision_monitor',
                #     name='collision_monitor',
                #     output='screen',
                #     respawn=use_respawn,
                #     respawn_delay=2.0,
                #     parameters=[configured_params],
                #     arguments=['--ros-args', '--log-level', log_level],
                #     remappings=remappings  # 보통 TF remap만 필요; cmd_vel in/out은 파라미터로 지정했음
                # ),        
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': navigation_lifecycle_nodes}],
                    on_exit=[EmitEvent(event=Shutdown())]
                    ),
            ]
        ),

        # Localization composable nodes
        LoadComposableNodes(
            condition=IfCondition(PythonExpression([use_composition, ' and not ', slam])),
            target_container=(namespace, '/', 'nav2_container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_map_server',
                    plugin='nav2_map_server::MapServer',
                    name='map_server',
                    parameters=[configured_params],
                    remappings=remappings),
                # ComposableNode(
                #     package='nav2_amcl',
                #     plugin='nav2_amcl::AmclNode',
                #     name='amcl',
                #     parameters=[configured_params],
                #     remappings=remappings),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_localization',
                    parameters=[{'use_sim_time': use_sim_time,
                                 'autostart': autostart,
                                 'node_names': localization_lifecycle_nodes}]),
            ],
        ),

        # Navigation composable nodes
        LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=(namespace, '/', 'nav2_container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_controller',
                    plugin='nav2_controller::ControllerServer',
                    name='controller_server',
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                ComposableNode(
                    package='nav2_smoother',
                    plugin='nav2_smoother::SmootherServer',
                    name='smoother_server',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_planner',
                    plugin='nav2_planner::PlannerServer',
                    name='planner_server',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='behavior_server',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_bt_navigator',
                    plugin='nav2_bt_navigator::BtNavigator',
                    name='bt_navigator',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_waypoint_follower',
                    plugin='nav2_waypoint_follower::WaypointFollower',
                    name='waypoint_follower',
                    parameters=[configured_params],
                    remappings=remappings),
                ComposableNode(
                    package='nav2_velocity_smoother',
                    plugin='nav2_velocity_smoother::VelocitySmoother',
                    name='velocity_smoother',
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
                # ComposableNode(
                #     package='nav2_route',
                #     plugin='nav2_route::RouteServer',
                #     name='route_server',
                #     parameters=[configured_params, {'graph_filepath': graph_filepath}],
                #     remappings=remappings),
                # ComposableNode(
                #     package='nav2_collision_monitor',
                #     plugin='nav2_collision_monitor::CollisionMonitor',
                #     name='collision_monitor',
                #     parameters=[configured_params],
                #     remappings=remappings
                # ),            
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{'use_sim_time': use_sim_time,
                                 'autostart': autostart,
                                 'node_names': navigation_lifecycle_nodes}]),
            ],
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the unified navigation group
    ld.add_action(unified_nav_cmd_group)

    return ld