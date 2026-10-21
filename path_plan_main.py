"""

Path planning

author: Nathan Duncan


"""

import math

import matplotlib.pyplot as plt

from environments import get_environment
from A-star import AStarPlanner

show_animation = True


class PlannerWorld():
    def __init__(self, ox, oy):
        """
        Initialize a grid map for search and planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        """

        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0


def main():
    print(__file__ + " start!!")

    ### Load Environment ###
    selected_env = '1b'  # Change to '1b' or pass custom settings as needed
    env = get_environment(selected_env)
    sx, sy = env.sx, env.sy
    gx, gy = env.gx, env.gy
    grid_size = env.grid_size
    ox, oy = env.ox, env.oy

    ### Plot Environment ###
    if show_animation:  # pragma: no cover
        ax = plt.gca()

        # Gridlines spaced by 1 unit at half numbers (-0.5, 0.5, 1.5, 2.5, ...)
        half_ticks_x = [i + 0.5 for i in range(-1, int(env.world_x) + 1)]
        half_ticks_y = [i + 0.5 for i in range(-1, int(env.world_y) + 1)]
        ax.set_xticks(half_ticks_x, minor=True)
        ax.set_yticks(half_ticks_y, minor=True)
        ax.grid(which="minor", color="lightgray", linestyle="-", linewidth=0.8)

        # Major ticks at integer coordinates to label cell centers (0, 1, 2, ...)
        ax.set_xticks(range(0, int(env.world_x) + 1))
        ax.set_yticks(range(0, int(env.world_y) + 1))
        ax.tick_params(which="minor", bottom=False, left=False)

        # Plot outer boundary matching the outer cell edges
        plt.plot(
            [-0.5, env.world_x + 0.5, env.world_x + 0.5, -0.5, -0.5],
            [-0.5, -0.5, env.world_y + 0.5, env.world_y + 0.5, -0.5],
            "-k",
            label="Boundary",
        )

        plt.plot(ox, oy, "sk", ms=12, mec='gray',label="Obstacles")
        plt.plot(sx, sy, "og", label="Start")
        plt.plot(gx, gy, "xb", label="Goal")
        plt.xlim(-0.5, env.world_x + 0.5)
        plt.ylim(-0.5, env.world_y + 0.5)
        plt.axis("equal")
        plt.show()

    ### Solve Planning Problem ###
    a_star = AStarPlanner(ox, oy, grid_size, show_animation)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    ### Plot Results ###
    # if show_animation:  # pragma: no cover
    #     plt.plot(rx, ry, "-r")
    #     plt.pause(0.001)
    #     plt.show()


if __name__ == '__main__':
    main()