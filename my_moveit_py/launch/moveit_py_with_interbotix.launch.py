from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os, yaml

from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
)

def _load_yaml(pkg, relpath):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def build_nodes(context, *args, **kwargs):
    lc_robot_model     = LaunchConfiguration('robot_model')
    lc_hardware_type   = LaunchConfiguration('hardware_type')
    lc_use_moveit_rviz = LaunchConfiguration('use_moveit_rviz')

    robot_model   = lc_robot_model.perform(context)         # e.g. "wx200"
    hardware_type = lc_hardware_type.perform(context)       # e.g. "gz_classic"
    use_sim_time  = (hardware_type in ('gz_classic', 'fake'))

    # 1) Bring up the Interbotix MoveIt stack
    interbotix_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'launch', 'xsarm_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': lc_robot_model,
            'hardware_type': lc_hardware_type,
            'use_moveit_rviz': lc_use_moveit_rviz,
        }.items(),
    )

    # 2) URDF & SRDF
    robot_description = {'robot_description': LaunchConfiguration('robot_description')}
    config_path = PathJoinSubstitution([FindPackageShare('interbotix_xsarm_moveit'), 'config'])
    robot_description_semantic = {
        'robot_description_semantic': construct_interbotix_xsarm_semantic_robot_description_command(
            robot_model=robot_model,
            config_path=config_path
        )
    }

    # 3) Kinematics (map under the exact key MoveIt expects)
    kin_yaml = _load_yaml('interbotix_xsarm_moveit', 'config/kinematics.yaml')
    if 'robot_description_kinematics' in kin_yaml:
        kinematics_config = {'robot_description_kinematics': kin_yaml['robot_description_kinematics']}
    else:
        kinematics_config = {'robot_description_kinematics': kin_yaml}

    # 4) OMPL pipeline config:
    #    - Explicitly set the planning plugin (prevents CHOMP fallback)
    #    - Add request adapters
    #    - Merge in detailed planner configs from ompl_planning.yaml
    ompl_planning_yaml = _load_yaml('interbotix_xsarm_moveit', 'config/ompl_planning.yaml')
    if 'ompl' in ompl_planning_yaml:
        ompl_detail = ompl_planning_yaml['ompl']
    else:
        ompl_detail = ompl_planning_yaml

    ompl_plugin_block = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
            # Merge detailed ompl planner settings
            **ompl_detail
        },
        'planning_pipelines': {
            'pipeline_names': ['ompl']
        }
    }

    # 5) Controller manager parameters for MoveItPy (same style as move_group)
    controllers_yaml = _load_yaml(
        'interbotix_xsarm_moveit',
        f'config/controllers/{robot_model}_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # 6) Your MoveItPy node’s own YAML (already ROS 2 schema)
    my_params_yaml = _load_yaml('my_moveit_py', 'config/moveit_py.yaml')
    if 'ros__parameters' in my_params_yaml:
        base_params = my_params_yaml['ros__parameters']
    elif 'moveit_py' in my_params_yaml and 'ros__parameters' in my_params_yaml['moveit_py']:
        base_params = my_params_yaml['moveit_py']['ros__parameters']
    else:
        base_params = my_params_yaml  # tolerate flat maps

    # 7) Topic + timeouts
    joint_states_topic = f'/{robot_model}/joint_states'

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    moveit_py_node = Node(
        package='my_moveit_py',
        executable='motion_planning_demo',
        name='moveit_py',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            base_params,
            ompl_plugin_block,
            robot_description,
            robot_description_semantic,
            kinematics_config,
            moveit_controllers,
            trajectory_execution_parameters,               # <--- add this
            {'planning_scene_monitor_options': {
                'joint_state_topic': joint_states_topic,
                'wait_for_initial_state_timeout': 30.0
            }},
        ],
    )

    return [interbotix_moveit_launch, moveit_py_node]

def generate_launch_description():
    _ = LaunchConfiguration('robot_model', default='wx200')
    _ = LaunchConfiguration('hardware_type', default='gz_classic')
    _ = LaunchConfiguration('use_moveit_rviz', default='true')
    return LaunchDescription([OpaqueFunction(function=build_nodes)])
