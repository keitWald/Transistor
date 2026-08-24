"""Task configuration for the active wheel-legged robot profile."""

from custom_wheel_legged_gym.envs.base.legged_robot_config import (
    LeggedRobotCfg,
    LeggedRobotCfgPPO,
)
from custom_wheel_legged_gym.envs.wheel_legged.robot_profile import ACTIVE_PROFILE


class CustomWheelLeggedCfg(LeggedRobotCfg):
    """Robot overrides; task-independent mechanics live in the base config."""

    robot_profile = ACTIVE_PROFILE

    class init_state(LeggedRobotCfg.init_state):
        pos = list(ACTIVE_PROFILE.initial_base_position)
        default_joint_angles = dict(ACTIVE_PROFILE.default_joint_angles)

    class control(LeggedRobotCfg.control):
        pos_action_scale = ACTIVE_PROFILE.position_action_scale
        vel_action_scale = ACTIVE_PROFILE.velocity_action_scale
        stiffness = dict(ACTIVE_PROFILE.stiffness)
        damping = dict(ACTIVE_PROFILE.damping)

    class asset(LeggedRobotCfg.asset):
        file = ACTIVE_PROFILE.urdf
        name = ACTIVE_PROFILE.name
        joint_roles = ACTIVE_PROFILE.roles
        kinematics = ACTIVE_PROFILE.kinematics
        offset = ACTIVE_PROFILE.kinematics.horizontal_offset
        l1 = ACTIVE_PROFILE.kinematics.upper_length
        l2 = ACTIVE_PROFILE.kinematics.lower_length
        penalize_contacts_on = []
        terminate_after_contacts_on = []
        self_collisions = 1
        flip_visual_attachments = False


class CustomWheelLeggedCfgPPO(LeggedRobotCfgPPO):
    class runner(LeggedRobotCfgPPO.runner):
        experiment_name = "custom_wheel_legged"
        max_iterations = 50000
