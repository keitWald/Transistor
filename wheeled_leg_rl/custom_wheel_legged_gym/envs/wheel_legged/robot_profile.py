"""Robot-specific parameters kept separate from the RL implementation.

Replace this profile when a new robot model is available. The environment
resolves joints by name at runtime, so URDF declaration order does not have to
match the logical policy order.
"""

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class JointRoles:
    left_hip: str
    left_knee: str
    left_wheel: str
    right_hip: str
    right_knee: str
    right_wheel: str

    @property
    def legs(self) -> Tuple[str, str, str, str]:
        return self.left_hip, self.left_knee, self.right_hip, self.right_knee

    @property
    def wheels(self) -> Tuple[str, str]:
        return self.left_wheel, self.right_wheel

    @property
    def policy_order(self) -> Tuple[str, ...]:
        return (
            self.left_hip,
            self.left_knee,
            self.left_wheel,
            self.right_hip,
            self.right_knee,
            self.right_wheel,
        )


@dataclass(frozen=True)
class LegKinematics:
    upper_length: float
    lower_length: float
    horizontal_offset: float = 0.0
    left_hip_sign: float = 1.0
    left_knee_sign: float = 1.0
    right_hip_sign: float = -1.0
    right_knee_sign: float = -1.0
    left_knee_offset: float = 1.5707963267948966
    right_knee_offset: float = 1.5707963267948966


@dataclass(frozen=True)
class RobotProfile:
    name: str
    urdf: str
    roles: JointRoles
    default_joint_angles: Dict[str, float]
    kinematics: LegKinematics
    initial_base_position: Tuple[float, float, float]
    stiffness: Dict[str, float]
    damping: Dict[str, float]
    position_action_scale: float
    velocity_action_scale: float

    def validate(self) -> None:
        role_names = self.roles.policy_order
        if len(set(role_names)) != 6:
            raise ValueError("The six joint roles must refer to six unique joints")
        missing = set(role_names) - set(self.default_joint_angles)
        if missing:
            raise ValueError(f"Missing default angles for joints: {sorted(missing)}")
        if self.kinematics.upper_length <= 0 or self.kinematics.lower_length <= 0:
            raise ValueError("Leg link lengths must be positive")
        if self.position_action_scale <= 0 or self.velocity_action_scale <= 0:
            raise ValueError("Action scales must be positive")


INFANTRY_V4 = RobotProfile(
    name="infantry_v4_reference",
    urdf=(
        "{CUSTOM_WHEEL_LEGGED_GYM_ROOT_DIR}/resources/robots/infantry_V4/"
        "urdf/infantry_V4_increase.urdf"
    ),
    roles=JointRoles(
        left_hip="lf0_Joint",
        left_knee="lf1_Joint",
        left_wheel="l_wheel_Joint",
        right_hip="rf0_Joint",
        right_knee="rf1_Joint",
        right_wheel="r_wheel_Joint",
    ),
    default_joint_angles={
        "lf0_Joint": 0.2,
        "lf1_Joint": 0.4,
        "l_wheel_Joint": 0.0,
        "rf0_Joint": -0.2,
        "rf1_Joint": -0.4,
        "r_wheel_Joint": 0.0,
    },
    kinematics=LegKinematics(upper_length=0.175, lower_length=0.208),
    initial_base_position=(0.0, 0.0, 0.1),
    stiffness={"f0": 20.0, "f1": 20.0, "wheel": 0.0},
    damping={"f0": 1.0, "f1": 1.0, "wheel": 0.2},
    position_action_scale=0.5,
    velocity_action_scale=10.0,
)

INFANTRY_V4.validate()
ACTIVE_PROFILE = INFANTRY_V4
