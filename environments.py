"""
Environment definitions and Environment class for path planning.

author: Nathan Duncan
"""

import math

# Environment dictionary settings
env_1a = {
    'world_x': 19,
    'world_y': 19,
    'obstacles': [
        [10, 5],
        [10, 6],
        [10, 7],
        [10, 8],
        [10, 9],
        [10, 10],
        [10, 11],
        [10, 12],
        [10, 13],
        [10, 14],
        [10, 15],
    ],
    'start': (5, 10),
    'goal': (15, 10),
}

env_1b = {
    'world_x': 19,
    'world_y': 19,
    'obstacles': [
        [4, 7],
        [5, 7],
        [6, 7],
        [7, 7],
        [8, 7],
        [9, 7],
        [10, 7],
        [10, 8],
        [10, 9],
        [10, 10],
        [10, 11],
        [10, 12],
        [10, 13],
        [9, 13],
        [8, 13],
        [7, 13],
        [6, 13],
        [5, 13],
        [4, 13],
    ],
    'start': (5, 10),
    'goal': (15, 10),
}

# Dictionary mapping setting names to their respective configurations
environments = {
    '1a': env_1a,
    '1b': env_1b,
}

class Environment:
    """
    Environment class that loads a selected setting (e.g. '1a', '1b' or a dict)
    and provides world dimensions, start/goal coordinates, obstacle lists,
    and a graph representation of the world grid.
    """

    def __init__(self, setting='1a', grid_size=1.0):
        if isinstance(setting, str):
            if setting not in environments:
                raise ValueError(
                    f"Unknown environment '{setting}'. Available options: {list(environments.keys())}"
                )
            self.name = setting
            data = environments[setting]
        elif isinstance(setting, dict):
            self.name = "custom"
            data = setting
        else:
            raise TypeError("setting must be an environment name string (e.g. '1a', '1b') or a dict")

        # World dimensions
        self.world_x = data.get('world_x', 19)
        self.world_y = data.get('world_y', 19)
        self.min_x = 0
        self.max_x = self.world_x
        self.min_y = 0
        self.max_y = self.world_y

        # Start and Goal positions
        start = data.get('start', (0, 0))
        self.sx = float(start[0])
        self.sy = float(start[1])

        goal = data.get('goal', (0, 0))
        self.gx = float(goal[0])
        self.gy = float(goal[1])

        # Obstacles
        self.obstacles = data.get('obstacles', [])
        self.ox = [float(pt[0]) for pt in self.obstacles]
        self.oy = [float(pt[1]) for pt in self.obstacles]

        # Grid and robot parameters
        self.grid_size = grid_size

    @staticmethod
    def get_edge_model():
        # dx, dy, cost
        edge = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return edge

    @property
    def start(self):
        return (self.sx, self.sy)

    @property
    def goal(self):
        return (self.gx, self.gy)

    @property
    def start_vertex(self):
        return self.graph.start

    @property
    def goal_vertex(self):
        return self.graph.goal

    def __getitem__(self, key):
        """Allow dictionary access e.g. env['start'], env['world_x'], env['graph']."""
        if hasattr(self, key):
            return getattr(self, key)
        raise KeyError(key)


def get_environment(setting='1a', grid_size=1.0):
    """Factory function to load an Environment instance."""
    return Environment(setting=setting, grid_size=grid_size)


if __name__ == '__main__':
    env = Environment('1a')
    print("Environment '1a' successfully loaded:")
    print(f"  World size: {env.world_x} x {env.world_y}")
    print(f"  Start: ({env.sx}, {env.sy}), Goal: ({env.gx}, {env.gy})")
    print(f"  Obstacles count: {len(env.ox)}")
