import torch
import torch.nn as nn
import numpy as np

class Controller():
    
    def __init__(self, input_size, hidden_size, output_size):

        linear1 = nn.Linear(input_size + hidden_size, hidden_size)
        linear2 = nn.Linear(hidden_size, output_size)

        self.linear1_w = linear1.weight.detach()
        self.linear1_b = linear1.bias.detach()
        self.linear2_w = linear2.weight.detach()
        self.linear2_b = linear2.bias.detach()

        self.hidden_act = nn.Tanh()
        self.output_act = nn.Tanh()
        
        self.hidden_size = hidden_size
        self.clear_hidden()
    
    def clear_hidden(self):
        self.prev_hidden = torch.zeros(self.hidden_size)
    
    def run_linear(self, x, w, b):
        return torch.nn.functional.linear(x, w, b) 

    def forward(self, x):

        input_vec = torch.cat((self.prev_hidden, x), 0)
        
        intermediate1 = self.run_linear(
            input_vec.unsqueeze(0),
            self.linear1_w,
            self.linear1_b
        )

        hidden = self.hidden_act(intermediate1)

        intermediate2 = self.run_linear(
            hidden,
            self.linear2_w,
            self.linear2_b
        )
        output = self.output_act(intermediate2)
        
        self.prev_hidden = hidden.squeeze(0)

        return output.squeeze(0)

    def mutate(self):
        weight1_noise = (torch.rand_like(self.linear1_w) - 0.5) / 10
        bias1_noise = (torch.rand_like(self.linear1_b) - 0.5) / 10

        weight2_noise = (torch.rand_like(self.linear2_w) - 0.5) / 10
        bias2_noise = (torch.rand_like(self.linear2_b) - 0.5) / 10

        self.linear1_w += weight1_noise
        self.linear1_b += bias1_noise

        self.linear2_w += weight2_noise
        self.linear2_b += bias2_noise

    # def __deepcopy__
