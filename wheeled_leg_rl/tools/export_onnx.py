"""Export the history encoder and actor from a training checkpoint to ONNX.

Run this on the training machine after installing PyTorch and ONNX support.
The script also writes deployment metadata next to the ONNX model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
import torch.nn as nn

from custom_wheel_legged_gym.rsl_rl.modules.actor_critic_sequence import (
    ActorCriticSequence,
)


class DeployablePolicy(nn.Module):
    def __init__(self, model: ActorCriticSequence):
        super().__init__()
        self.encoder = model.encoder
        self.actor = model.actor

    def forward(self, obs, obs_history):
        latent = self.encoder(obs_history)
        return self.actor(torch.cat((obs, latent), dim=-1))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--opset", type=int, default=17)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    metadata = checkpoint.get("robot_metadata") or {}
    num_obs = int(metadata.get("num_observations", 25))
    history_length = int(metadata.get("observation_history_length", 5))
    num_actions = int(metadata.get("num_actions", 6))
    latent_dim = 3

    model = ActorCriticSequence(
        num_obs=num_obs,
        num_critic_obs=1,
        num_actions=num_actions,
        num_encoder_obs=num_obs * history_length,
        latent_dim=latent_dim,
        encoder_hidden_dims=[128, 64],
        actor_hidden_dims=[128, 64, 32],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )
    state = {
        key: value
        for key, value in checkpoint["model_state_dict"].items()
        if not key.startswith("critic.")
    }
    model.load_state_dict(state, strict=False)
    policy = DeployablePolicy(model).eval()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    dummy_obs = torch.zeros(1, num_obs)
    dummy_history = torch.zeros(1, num_obs * history_length)
    torch.onnx.export(
        policy,
        (dummy_obs, dummy_history),
        args.output,
        opset_version=args.opset,
        input_names=["obs", "obs_history"],
        output_names=["actions"],
        dynamic_axes={
            "obs": {0: "batch"},
            "obs_history": {0: "batch"},
            "actions": {0: "batch"},
        },
    )

    metadata.update(
        {
            "num_observations": num_obs,
            "observation_history_length": history_length,
            "num_actions": num_actions,
            "latent_dim": latent_dim,
            "onnx_inputs": ["obs", "obs_history"],
            "onnx_output": "actions",
        }
    )
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"ONNX: {args.output}")
    print(f"Metadata: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
