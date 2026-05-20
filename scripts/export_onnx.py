import argparse
import os
import sys
import copy
import torch
import torch.onnx

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'rsl_rl'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from rsl_rl.modules import ActorCriticDreamWaQ


class DreamWaQONNXExporter(torch.nn.Module):
    def __init__(self, actor_critic):
        super().__init__()
        self.actor = copy.deepcopy(actor_critic.actor)
        self.vae = copy.deepcopy(actor_critic.vae)
        self.actor.eval()
        self.vae.eval()

    def forward(self, obs_history):
        code, _, _, _ = self.vae.cenet_forward(obs_history)
        actor_input = torch.cat((code, obs_history[:, -45:]), dim=1)
        actions_mean = self.actor(actor_input)
        return actions_mean

    def export(self, path, opset=11):
        os.makedirs(path, exist_ok=True)
        onnx_file = os.path.join(path, 'policy.onnx')

        dummy_obs = torch.randn(1, 225)

        self.to('cpu')
        self.eval()
        with torch.no_grad():
            torch.onnx.export(
                self,
                (dummy_obs,),
                onnx_file,
                input_names=['obs'],
                output_names=['actions'],
                dynamic_axes={
                    'obs': {0: 'batch'},
                    'actions': {0: 'batch'}
                },
                opset_version=opset
            )
        print(f'ONNX model exported to: {onnx_file}')


def main():
    parser = argparse.ArgumentParser(description='Export DreamWaQ checkpoint to ONNX')
    parser.add_argument('--checkpoint', type=str, required=True,
                        help='Path to model checkpoint (.pt)')
    parser.add_argument('--output-dir', type=str, required=True,
                        help='Output directory for ONNX file')
    args = parser.parse_args()

    device = 'cpu'

    actor_critic = ActorCriticDreamWaQ(
        num_actor_obs=45,
        num_critic_obs=235,
        num_actions=12,
        cenet_in_dim=225,
        num_latent_dims=16,
        num_explicit_dims=3,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation='elu',
        init_noise_std=1.0,
    ).to(device)

    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=True)
    actor_critic.load_state_dict(checkpoint['model_state_dict'])
    print(f'Loaded checkpoint: {args.checkpoint}')
    print(f'  iteration: {checkpoint.get("iter", "unknown")}')

    exporter = DreamWaQONNXExporter(actor_critic)
    exporter.export(args.output_dir)


if __name__ == '__main__':
    main()
