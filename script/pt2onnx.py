import torch
import torch.onnx

# Define your PPO model class
class PPOModel(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(PPOModel, self).__init__()
        # Define the network architecture
        self.actor = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim)  # Output dimensions for actions
        )
        
        self.critic = nn.Sequential(
            nn.Linear(input_dim, 768),
            nn.ReLU(),
            nn.Linear(768, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1)  # Output a single value for state value
        )
    
    def forward(self, x):
        action_probs = self.actor(x)
        state_value = self.critic(x)
        return action_probs, state_value

# Load your trained PyTorch model
def load_model(pt_path, input_dim, output_dim):
    model = PPOModel(input_dim, output_dim)
    model.load_state_dict(torch.load(pt_path))
    model.eval()  # Set the model to evaluation mode
    return model

# Define the input dimension and output dimension
input_dim = 615
output_dim = 10
pt_path = "path_to_your_trained_model.pt"  # Replace with your .pt file path

# Load the model
model = load_model(pt_path, input_dim, output_dim)

# Define a dummy input tensor with the same shape as your model's input
dummy_input = torch.randn(1, input_dim)

# Export the model to ONNX format
onnx_path = "ppo_model.onnx"
torch.onnx.export(model, dummy_input, onnx_path, verbose=True, input_names=['input'], output_names=['action_probs', 'state_value'])

print(f"Model successfully exported to {onnx_path}")
