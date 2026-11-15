"""Compact spatial and range features with an explicit motion/goal branch."""

import torch
from torch import nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class NavigationFeatures(BaseFeaturesExtractor):
    def __init__(self, observation_space):
        super().__init__(observation_space, features_dim=80)
        self.map_encoder = nn.Sequential(
            nn.Conv2d(3, 8, kernel_size=3, stride=2), nn.ReLU(),
            nn.Conv2d(8, 16, kernel_size=3, stride=2), nn.ReLU(),
            nn.Flatten(), nn.Linear(16 * 6 * 6, 32), nn.ReLU())
        self.range_encoder = nn.Sequential(nn.Linear(288, 32), nn.ReLU())
        self.state_encoder = nn.Sequential(nn.Linear(9, 16), nn.ReLU())

    def forward(self, observations):
        maps = observations[:, :2700].reshape(-1, 3, 30, 30)
        ranges = observations[:, 2700:2988]
        state = observations[:, 2988:]
        return torch.cat((self.map_encoder(maps), self.range_encoder(ranges), self.state_encoder(state)), dim=1)
