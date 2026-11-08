"""
Continuous Environment definitions and Environment class for path planning.

author: Nathan Duncan
"""

import math

# Environment dictionary settings
""" Figure 1, l = 25 """
env_1_l25 = {
    'world_x': 100,
    'world_y': 100,
    'obstacles': [
        ((45, 55), (25, 100-25))
    ],
    'start': (25, 50),
    'goal': (75, 50),
}

""" Figure 1, l = 4 """
env_1_l4 = {
    'world_x': 100,
    'world_y': 100,
    'obstacles': [
        ((45, 55), (4, 100-4))
    ],
    'start': (25, 50),
    'goal': (75, 50),
}

""" Figure 2a """
env_2a = {
    'world_x': 100,
    'world_y': 100,
    'obstacles': [
        ((10, 15), (35, 49)),
        ((15, 35), (35, 40)),
        ((35, 40), (35, 65)),
        ((15, 35), (60, 65)),
        ((10, 15), (51, 65))
    ],
    'start': (25, 50),
    'goal': (75, 50),
}

""" Figure 2b """
env_2b = {
    'world_x': 100,
    'world_y': 100,
    'obstacles': [
        ((85, 90), (35, 49)),
        ((65, 85), (35, 40)),
        ((60, 65), (35, 65)),
        ((65, 85), (60, 65)),
        ((85, 90), (51, 65))
    ],
    'start': (25, 50),
    'goal': (75, 50),
}

""" Figure 2c """
env_2c = {
    'world_x': 100,
    'world_y': 100,
    'obstacles': [
        ((10, 15), (35, 49)),
        ((15, 35), (35, 40)),
        ((35, 40), (35, 65)),
        ((15, 35), (60, 65)),
        ((10, 15), (51, 65)),

        ((85, 90), (35, 49)),
        ((65, 85), (35, 40)),
        ((60, 65), (35, 65)),
        ((65, 85), (60, 65)),
        ((85, 90), (51, 65)),
    ],
    'start': (25, 50),
    'goal': (75, 50),
}

# Dictionary mapping setting names to their respective configurations
environments = {
    '1_l25': env_1_l25,
    '1_l4': env_1_l4,
    '2a': env_2a,
    '2b': env_2b,
    '2c': env_2c,
}

class Environment:
    """
    Environment class that loads a selected setting (e.g. '1a', '1b' or a dict)
    and provides world dimensions, start/goal coordinates, obstacle lists,
    and a graph representation of the world grid.
    """

    def __init__(self, setting='1_l25'):
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
            raise TypeError("setting must be an environment name string or a dict")

        # Workspace limits
        self.min_x = 0.0
        self.max_x = float(data.get('world_x', 100))
        self.min_y = 0.0
        self.max_y = float(data.get('world_y', 100))

        # Start and Goal
        start = data.get('start', (0.0, 0.0))
        self.sx = float(start[0])
        self.sy = float(start[1])

        goal = data.get('goal', (0.0, 0.0))
        self.gx = float(goal[0])
        self.gy = float(goal[1])

        # Obstacles: list of ((minx, maxx), (miny, maxy))
        self.obstacles = data.get('obstacles', [])

    @property
    def start(self):
        return (self.sx, self.sy)

    @property
    def goal(self):
        return (self.gx, self.gy)

    @property
    def bounds(self):
        return (self.min_x, self.max_x, self.min_y, self.max_y)

    def __getitem__(self, key):
        if hasattr(self, key):
            return getattr(self, key)
        raise KeyError(key)


def get_environment(setting='1_l25'):
    return Environment(setting=setting)


if __name__ == '__main__':
    env = Environment('1_l25')
    print(f"Loaded environment: {env.name}")
    print(f"Bounds: X[{env.min_x}, {env.max_x}], Y[{env.min_y}, {env.max_y}]")
    print(f"Start: {env.start}, Goal: {env.goal}")
    print(f"Obstacles: {env.obstacles}")