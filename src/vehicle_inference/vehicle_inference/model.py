import torch.nn as nn


class NvidiaModel(nn.Module):
    def __init__(self):
        super().__init__()

        # define layers using nn.Sequential
        self.conv_layers = nn.Sequential(
            # first convolutional layer
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.BatchNorm2d(24),
            nn.ReLU(),

            # second convolutional layer
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.BatchNorm2d(36),
            nn.ReLU(),

            # third convolutional layer
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.BatchNorm2d(48),
            nn.ReLU(),

            # fourth convolutional layer
            nn.Conv2d(48, 64, kernel_size=3),
            nn.BatchNorm2d(64),
            nn.ReLU(),

            # fifth convolutional layer
            nn.Conv2d(64, 64, kernel_size=3),
            nn.BatchNorm2d(64),
            nn.ReLU(),
        )

        self.flat_layers = nn.Sequential(
            # flatten
            nn.Flatten(),
            nn.Dropout(p=0.5),

            # first fully connected layer
            nn.Linear(1152, 1164),
            nn.BatchNorm1d(1164),
            nn.ReLU(),

            # second fully connected layer
            nn.Linear(1164, 100),
            nn.BatchNorm1d(100),
            nn.ReLU(),

            # third fully connected layer
            nn.Linear(100, 50),
            nn.BatchNorm1d(50),
            nn.ReLU(),

            # fourth fully connected layer
            nn.Linear(50, 10),

            # output layer
            nn.Linear(10, 1)
        )

    def forward(self, x):
        x = self.conv_layers(x)
        x = self.flat_layers(x)
        return x.squeeze()