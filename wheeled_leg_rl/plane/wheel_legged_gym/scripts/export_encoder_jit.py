import argparse
import os
import torch
import torch.nn as nn

parser = argparse.ArgumentParser(description="Export the history encoder as TorchScript")
parser.add_argument("--checkpoint", required=True, help="Path to model_*.pt")
parser.add_argument("--out", default="export_onnx/encoder.pt", help="Output .pt path")
args = parser.parse_args()

device = "cpu"
ckpt = torch.load(args.checkpoint, map_location=device)
sd = ckpt["model_state_dict"]

# Infer dimensions from the checkpoint so the exporter stays in sync with config.
encoder = nn.Sequential(
    nn.Linear(sd["encoder.0.weight"].shape[1], sd["encoder.0.weight"].shape[0]),
    nn.ELU(),
    nn.Linear(sd["encoder.0.weight"].shape[0], sd["encoder.2.weight"].shape[0]),
    nn.ELU(),
    nn.Linear(sd["encoder.2.weight"].shape[0], sd["encoder.4.weight"].shape[0]),
).to(device).eval()

# 从 checkpoint 填权重：你 ckpt 里就是 encoder.0 / encoder.2 / encoder.4
with torch.no_grad():
    encoder[0].weight.copy_(sd["encoder.0.weight"])
    encoder[0].bias.copy_(sd["encoder.0.bias"])
    encoder[2].weight.copy_(sd["encoder.2.weight"])
    encoder[2].bias.copy_(sd["encoder.2.bias"])
    encoder[4].weight.copy_(sd["encoder.4.weight"])
    encoder[4].bias.copy_(sd["encoder.4.bias"])

# 导出 TorchScript
out_dir = os.path.dirname(os.path.abspath(args.out))
os.makedirs(out_dir, exist_ok=True)
dummy = torch.zeros(1, encoder[0].in_features, dtype=torch.float32)
jit_enc = torch.jit.trace(encoder, dummy)
jit_enc.save(args.out)

print("Saved:", os.path.abspath(args.out))
print("schema:", jit_enc.forward.schema)
