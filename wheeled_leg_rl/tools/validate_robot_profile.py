"""Validate a RobotProfile and URDF without importing Isaac Gym or PyTorch."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = (
    PROJECT_ROOT
    / "custom_wheel_legged_gym"
    / "envs"
    / "wheel_legged"
    / "robot_profile.py"
)


def load_profile_module():
    spec = importlib.util.spec_from_file_location("standalone_robot_profile", PROFILE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load profile module: {PROFILE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def resolve_urdf_path(template: str) -> Path:
    resolved = template.replace(
        "{CUSTOM_WHEEL_LEGGED_GYM_ROOT_DIR}", str(PROJECT_ROOT)
    )
    return Path(resolved).resolve()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        help="Optional URDF override. Defaults to ACTIVE_PROFILE.urdf.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profile_module = load_profile_module()
    profile = profile_module.ACTIVE_PROFILE
    profile.validate()

    urdf_path = args.urdf.resolve() if args.urdf else resolve_urdf_path(profile.urdf)
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")

    root = ET.parse(urdf_path).getroot()
    joints = {
        joint.attrib["name"]: joint
        for joint in root.findall("joint")
        if joint.attrib.get("type") != "fixed"
    }
    role_names = profile.roles.policy_order
    missing = [name for name in role_names if name not in joints]
    extra = [name for name in joints if name not in role_names]
    if missing:
        raise ValueError(f"Profile joints missing from URDF: {missing}")
    if extra:
        raise ValueError(
            "The current six-action environment does not support additional movable "
            f"joints: {extra}"
        )

    print(f"Profile: {profile.name}")
    print(f"URDF: {urdf_path}")
    print("Logical policy roles:")
    for role, name in zip(
        ("left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"),
        role_names,
    ):
        limit = joints[name].find("limit")
        attrs = limit.attrib if limit is not None else {}
        print(
            f"  {role:>11}: {name:<18} "
            f"effort={attrs.get('effort', 'n/a'):<8} "
            f"velocity={attrs.get('velocity', 'n/a')}"
        )

    print("Validation passed: profile and URDF define the expected six joints.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
