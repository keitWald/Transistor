"""Dependency-light ONNX policy and mixed leg-position/wheel-speed controller."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Dict

import numpy as np


class PolicyController:
    """Construct training-aligned observations and return named joint torques.

    Joint measurements are accepted as name-to-value mappings, so the hardware
    or simulator order may differ from the Isaac Gym order stored in metadata.
    """

    def __init__(
        self,
        onnx_path: str | Path,
        metadata_path: str | Path | None = None,
        providers: Sequence[str] | None = None,
    ) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError(
                "onnxruntime is required only on the deployment machine"
            ) from exc

        self.onnx_path = Path(onnx_path)
        metadata_file = (
            Path(metadata_path) if metadata_path else self.onnx_path.with_suffix(".json")
        )
        self.meta = json.loads(metadata_file.read_text(encoding="utf-8"))
        self._validate_metadata()

        selected_providers = list(providers) if providers else ["CPUExecutionProvider"]
        self.session = ort.InferenceSession(
            str(self.onnx_path), providers=selected_providers
        )
        inputs = {item.name for item in self.session.get_inputs()}
        if not {"obs", "obs_history"}.issubset(inputs):
            raise ValueError(f"Unexpected ONNX inputs: {sorted(inputs)}")

        self.dof_names = tuple(self.meta["dof_names"])
        self.leg_indices = np.asarray(self.meta["leg_dof_indices"], dtype=np.int64)
        self.wheel_indices = np.asarray(self.meta["wheel_dof_indices"], dtype=np.int64)
        self.default_q = np.asarray(self.meta["default_dof_pos"], dtype=np.float32)
        self.kp = np.asarray(self.meta["nominal_stiffness"], dtype=np.float32)
        self.kd = np.asarray(self.meta["nominal_damping"], dtype=np.float32)
        self.torque_limits = np.asarray(self.meta["torque_limits"], dtype=np.float32)
        self.num_obs = int(self.meta["num_observations"])
        self.history_length = int(self.meta["observation_history_length"])
        self.last_actions = np.zeros(len(self.dof_names), dtype=np.float32)
        self.history: np.ndarray | None = None

    def _validate_metadata(self) -> None:
        required = {
            "dof_names",
            "leg_dof_indices",
            "wheel_dof_indices",
            "default_dof_pos",
            "nominal_stiffness",
            "nominal_damping",
            "torque_limits",
            "num_observations",
            "observation_history_length",
            "position_action_scale",
            "velocity_action_scale",
            "observation_scales",
        }
        missing = required - set(self.meta)
        if missing:
            raise ValueError(f"Deployment metadata is missing: {sorted(missing)}")

    def reset(self) -> None:
        self.last_actions.fill(0.0)
        self.history = None

    def _named_vector(self, values: Mapping[str, float], label: str) -> np.ndarray:
        missing = [name for name in self.dof_names if name not in values]
        if missing:
            raise ValueError(f"{label} is missing joints: {missing}")
        return np.asarray([values[name] for name in self.dof_names], dtype=np.float32)

    def build_observation(
        self,
        base_angular_velocity: Sequence[float],
        projected_gravity: Sequence[float],
        command: Sequence[float],
        joint_position: Mapping[str, float],
        joint_velocity: Mapping[str, float],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        q = self._named_vector(joint_position, "joint_position")
        qd = self._named_vector(joint_velocity, "joint_velocity")
        scales = self.meta["observation_scales"]
        command_scale = np.asarray(
            [
                scales["linear_velocity"],
                scales["angular_velocity"],
                scales["height_command"],
            ],
            dtype=np.float32,
        )
        obs = np.concatenate(
            (
                np.asarray(base_angular_velocity, dtype=np.float32)
                * float(scales["angular_velocity"]),
                np.asarray(projected_gravity, dtype=np.float32),
                np.asarray(command, dtype=np.float32) * command_scale,
                (q[self.leg_indices] - self.default_q[self.leg_indices])
                * float(scales["joint_position"]),
                qd * float(scales["joint_velocity"]),
                self.last_actions,
            )
        ).astype(np.float32)
        if obs.shape != (self.num_obs,):
            raise ValueError(f"Observation has shape {obs.shape}, expected {(self.num_obs,)}")
        clip_obs = float(self.meta.get("clip_observations", 100.0))
        obs = np.clip(obs, -clip_obs, clip_obs)

        if self.history is None:
            self.history = np.repeat(obs[None, :], self.history_length, axis=0)
        else:
            self.history[:-1] = self.history[1:]
            self.history[-1] = obs
        return obs, self.history.reshape(-1).copy(), qd

    def step(
        self,
        base_angular_velocity: Sequence[float],
        projected_gravity: Sequence[float],
        command: Sequence[float],
        joint_position: Mapping[str, float],
        joint_velocity: Mapping[str, float],
    ) -> Dict[str, float]:
        obs, history, qd = self.build_observation(
            base_angular_velocity,
            projected_gravity,
            command,
            joint_position,
            joint_velocity,
        )
        output = self.session.run(
            ["actions"],
            {"obs": obs[None, :], "obs_history": history[None, :]},
        )[0]
        actions = np.asarray(output, dtype=np.float32).reshape(-1)
        if actions.shape != (len(self.dof_names),):
            raise ValueError(f"Policy returned invalid action shape: {actions.shape}")
        clip_actions = float(self.meta.get("clip_actions", 100.0))
        actions = np.clip(actions, -clip_actions, clip_actions)

        q = self._named_vector(joint_position, "joint_position")
        position_ref = actions * float(self.meta["position_action_scale"])
        position_ref[self.wheel_indices] = 0.0
        velocity_ref = actions * float(self.meta["velocity_action_scale"])
        velocity_ref[self.leg_indices] = 0.0
        torque = self.kp * (position_ref + self.default_q - q) + self.kd * (
            velocity_ref - qd
        )
        torque = np.clip(torque, -self.torque_limits, self.torque_limits)
        self.last_actions[:] = actions
        return {name: float(torque[i]) for i, name in enumerate(self.dof_names)}
