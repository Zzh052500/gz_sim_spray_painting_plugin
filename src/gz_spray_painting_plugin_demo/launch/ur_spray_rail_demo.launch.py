"""
ur_spray_rail_demo.launch.py
=============================
Rail-mounted variant of ur_spray_demo.launch.py: the UR5e + spray tool
rides on a carriage that slides along a linear rail instead of sitting on a
fixed pedestal, mimicking a real gantry-mounted spray-paint cell for
objects too large for the arm's own reach (cars, aircraft panels).

Structurally identical to ur_spray_demo.launch.py (same node graph, same
staggered spawn timing) except for:
  * robot description : ur_spray_gz_rail.urdf.xacro (adds rail_to_carriage)
  * controllers        : ur_sim_controllers_rail.yaml (adds
                          rail_trajectory_controller, independent from the
                          arm's joint_trajectory_controller)
  * world              : demo_car_rail.sdf (same car, no pedestal - the
                          rail assembly is its own support)
  * spawn pose         : z=0 (ground level) - the rail + support post
                          height is now baked into the URDF itself, unlike
                          the fixed-base demo which spawns at z=0.80 to sit
                          on top of a separate static pedestal model.

The rail is driven independently of MoveIt: rail_spray_demo.py positions it
before each spray pass via rail_trajectory_controller, while the arm's
6-DOF "ur_manipulator" MoveIt planning group is unchanged from the
fixed-base demo.

Usage:
  ros2 launch gz_spray_painting_plugin_demo ur_spray_rail_demo.launch.py
  ros2 launch gz_spray_painting_plugin_demo ur_spray_rail_demo.launch.py headless:=true
  ros2 launch gz_spray_painting_plugin_demo ur_spray_rail_demo.launch.py rail_length:=4.0
"""

import os
import re
import subprocess
import traceback

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    headless             = LaunchConfiguration("headless")
    paint_interval_steps = LaunchConfiguration("paint_interval_steps").perform(context)
    perf_log_path        = LaunchConfiguration("perf_log_path").perform(context)
    rail_length           = LaunchConfiguration("rail_length").perform(context)

    # Plugin package: libSprayPaintPlugin.so
    spray_pkg_prefix = get_package_prefix("gz_sim_spray_painting_plugin")

    # Demo package: URDF, configs, worlds, models, RViz
    demo_pkg_share = get_package_share_directory("gz_spray_painting_plugin_demo")

    description_file = os.path.join(demo_pkg_share, "urdf", "ur_spray_gz_rail.urdf.xacro")
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("gz_spray_painting_plugin_demo"), "config", "ur_sim_controllers_rail.yaml"]
    ).perform(context)

    # ── Build robot description URDF ─────────────────────────────────────────
    xacro_cmd = [
        "xacro", description_file,
        "ur_type:=ur5e",
        f"simulation_controllers:={controllers_file}",
        "safety_limits:=true",
        "name:=ur",
        "tf_prefix:=",
        f"paint_interval_steps:={paint_interval_steps}",
        f"perf_log_path:={perf_log_path}",
        f"rail_length:={rail_length}",
    ]
    xacro_env = os.environ.copy()
    demo_pkg_prefix = "/ws/install/gz_spray_painting_plugin_demo"
    if os.path.isdir(demo_pkg_prefix):
        existing = xacro_env.get("AMENT_PREFIX_PATH", "")
        xacro_env["AMENT_PREFIX_PATH"] = f"{demo_pkg_prefix}:{existing}" if existing else demo_pkg_prefix
    urdf_str = subprocess.check_output(xacro_cmd, env=xacro_env, stderr=subprocess.PIPE).decode()
    robot_description = {"robot_description": ParameterValue(urdf_str, value_type=str)}

    # ── Gazebo environment ────────────────────────────────────────────────────
    set_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            os.path.join(spray_pkg_prefix, "lib", "gz_sim_spray_painting_plugin"),
            ":/ws/install/gz_ros2_control/lib:",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.dirname(demo_pkg_share) + ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    world_path = os.path.join(demo_pkg_share, "worlds", "demo_car_rail.sdf")

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r", "-v", "4"],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", world_path, "-r", "-v", "4"],
        output="screen",
        condition=IfCondition(headless),
    )

    # ── ros_gz_bridge ─────────────────────────────────────────────────────────
    bridge_config = os.path.join(demo_pkg_share, "config", "ros_gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # ── joint_state_publisher (initial pose until JSB starts at T+20 s) ──────
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            robot_description,
            {
                "use_sim_time": True,
                "rate": 10,
                "zeros.shoulder_lift_joint": -1.5708,
                "zeros.elbow_joint":          1.5708,
                "zeros.wrist_1_joint":        -1.5708,
                "zeros.wrist_2_joint":        -1.5708,
                "zeros.rail_to_carriage":      0.0,
            },
        ],
    )

    # ── Spawn robot at T+8 s ──────────────────────────────────────────────────
    # z=0: the rail track sits directly on the ground, and the support post
    # that the fixed-base demo gets from a separate pedestal model is now
    # baked into the URDF's rail_carriage link (see rail_axis.xacro),
    # already accounted for by the xacro's own origin offset onto base_link.
    urdf_tmp = "/tmp/ur_spray_rail_generated.urdf"
    with open(urdf_tmp, "w") as f:
        f.write(urdf_str)

    spawn_robot = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(
            cmd=[
                "gz", "service",
                "-s", "/world/demo_car_rail/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "30000",
                "--req",
                (
                    f'sdf_filename: "{urdf_tmp}" '
                    f'name: "ur" '
                    f'allow_renaming: false '
                    f'pose: {{ position: {{x: 0.0 y: 0.0 z: 0.0}} '
                    f'orientation: {{x: 0 y: 0 z: 0.7071 w: 0.7071}} }}'
                ),
            ],
            output="screen",
        )],
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    joint_state_broadcaster_spawner = TimerAction(
        period=20.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )],
    )
    joint_trajectory_controller_spawner = TimerAction(
        period=25.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "-c", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )],
    )
    # Staggered a couple seconds after the arm's controller so the two
    # `controller_manager/spawner` processes don't race the same service.
    rail_trajectory_controller_spawner = TimerAction(
        period=27.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "rail_trajectory_controller",
                "-c", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )],
    )

    # ── MoveIt 2 ─────────────────────────────────────────────────────────────
    # Unchanged from ur_spray_demo.launch.py: the SRDF chain only ever knew
    # about the 6 arm joints (base_link -> tool0, tip patched to the nozzle
    # below), so it needs no changes for the rail — the rail sits entirely
    # outside this planning group and is driven separately, above.
    moveit_actions = []
    try:
        from ur_moveit_config.launch_common import load_yaml  # noqa: PLC0415

        moveit_pkg_share = get_package_share_directory("ur_moveit_config")

        srdf_xacro = os.path.join(moveit_pkg_share, "srdf", "ur.srdf.xacro")
        srdf_str = subprocess.check_output(
            ["xacro", srdf_xacro, "name:=ur", "prefix:="],
            env=xacro_env, stderr=subprocess.PIPE,
        ).decode()
        # Redirect the planning chain tip to the spray nozzle instead of tool0
        srdf_str = re.sub(
            r'(<chain\b[^>]*\btip_link=")[^"]+(")',
            r'\1spray_gun_nozzle_link\2',
            srdf_str,
        )
        robot_description_semantic = {
            "robot_description_semantic": ParameterValue(srdf_str, value_type=str)
        }

        kinematics_yaml    = load_yaml("gz_spray_painting_plugin_demo", "config/kinematics.yaml")
        joint_limits_yaml  = load_yaml("ur_moveit_config", "config/joint_limits.yaml")
        ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
        controllers_yaml   = load_yaml("ur_moveit_config", "config/controllers.yaml")

        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

        ompl_pipeline = {
            "move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": (
                    "default_planner_request_adapters/AddTimeOptimalParameterization "
                    "default_planner_request_adapters/FixWorkspaceBounds "
                    "default_planner_request_adapters/FixStartStateBounds "
                    "default_planner_request_adapters/FixStartStateCollision "
                    "default_planner_request_adapters/FixStartStatePathConstraints"
                ),
                "start_state_max_bounds_error": 0.1,
            }
        }
        ompl_pipeline["move_group"].update(ompl_planning_yaml)

        moveit_controllers = {
            "moveit_simple_controller_manager": controllers_yaml,
            "moveit_controller_manager":
                "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }
        trajectory_execution = {
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
            "trajectory_execution.execution_duration_monitoring": False,
        }
        planning_scene_monitor = {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }

        common_params = [
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            ompl_pipeline,
            {"use_sim_time": True},
        ]

        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=common_params + [
                {"publish_robot_description_semantic": True},
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor,
            ],
        )

        moveit_actions = [move_group_node]

    except Exception as exc:
        print(f"\n[ur_spray_rail_demo] MoveIt failed to load, skipping.\n"
              f"  Error: {exc}\n{traceback.format_exc()}")

    return [
        set_plugin_path,
        set_resource_path,
        gazebo,
        gazebo_headless,
        ros_gz_bridge,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        rail_trajectory_controller_spawner,
        *moveit_actions,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server only (no GUI).",
        ),
        DeclareLaunchArgument(
            "paint_interval_steps",
            default_value="10",
            description="Run the spray ray-scan every N simulation steps "
                        "(1 = no rate-limiting, scan every tick).",
        ),
        DeclareLaunchArgument(
            "perf_log_path",
            default_value="/ws/file_logs/spray_perf_rail.csv",
            description="CSV path for per-scan plugin timing "
                        "(sim_time_s,paint_interval_steps,num_rays,scan_us,valid_hits,patches_created). "
                        "Empty disables perf logging.",
        ),
        DeclareLaunchArgument(
            "rail_length",
            default_value="3.0",
            description="Total travel length of the rail (m). The carriage "
                        "can move from -rail_length/2 to +rail_length/2.",
        ),
        SetEnvironmentVariable(name="GZ_VERSION", value="harmonic"),
        OpaqueFunction(function=launch_setup),
    ])
