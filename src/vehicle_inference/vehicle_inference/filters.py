import numpy as np


class AverageFilter:
    def __init__(self, max_size=10):
        self.max_size = max_size
        self.data = np.array([])

    def add(self, value):
        self.data = np.append(self.data, value)
        if len(self.data) > self.max_size:
            self.data = np.delete(self.data, 0)

    def get_average(self):
        return np.average(self.data) if len(self.data) else 0
