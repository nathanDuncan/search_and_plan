"""

Path planning

author: Nathan Duncan


"""

import math

import matplotlib.pyplot as plt

from environments import get_environment
from AStar import AStarPlanner

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
    selected_env = '1b'
    flip_start_goal = False
    heuristic = "euclidean"

    env = get_environment(selected_env)
    sx, sy = int(env.sx), int(env.sy)
    gx, gy = int(env.gx), int(env.gy)
    ox, oy = [int(x) for x in env.ox], [int(y) for y in env.oy]
    obstacle_set = set(zip(ox, oy))

    # Define explicit world bounds
    min_x, max_x = 1, int(env.world_x)
    min_y, max_y = 1, int(env.world_y)

    ### Pre-plot Canvas for Animation ###
    fig, ax = plt.subplots(figsize=(8, 8))

    # Free space nodes (black dots) within bounds
    free_x, free_y = [], []
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            if (x, y) not in obstacle_set:
                free_x.append(x)
                free_y.append(y)

    ax.scatter(free_x, free_y, c='grey', s=20, zorder=1, label="Free Nodes")
    ax.scatter(ox, oy, c='white', edgecolors='black', linewidths=1.2, s=35, zorder=4, label="Obstacles")

    # Start and Goal markers
    ax.scatter(sx, sy, c='green', marker='x', s=120, linewidths=3, zorder=10, label="Start")
    ax.scatter(gx, gy, c='red', marker='x', s=120, linewidths=3, zorder=10, label="Goal")

    ## Outer boundary adjusted to min_x and min_y
    ax.plot(
        [min_x - 0.5, max_x + 0.5, max_x + 0.5, min_x - 0.5, min_x - 0.5],
        [min_y - 0.5, min_y - 0.5, max_y + 0.5, max_y + 0.5, min_y - 0.5],
        "-k", linewidth=1.5, zorder=1
    )

    # Axis limits and ticks starting from min_x and min_y
    ax.set_xticks(range(min_x, max_x + 1))
    ax.set_yticks(range(min_y, max_y + 1))
    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    ax.set_aspect("equal")

    # Connect escape key to quit animation early
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])

    ### Solve Planning Problem ###
    a_star = AStarPlanner(
        ox=ox,
        oy=oy,
        edge_model=env.edge_model,
        heuristic=heuristic,
        show_animation=show_animation,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
    )
    rx, ry = a_star.planning(sx, sy, gx, gy)

    ### Plot Final Path ###
    if rx and ry:
        ax.plot(rx, ry, color='red', linewidth=2.5, zorder=5, label="Final Path")
        ax.scatter(rx, ry, c='red', s=45, zorder=6, label="Path Nodes")
        # ax.legend(loc='upper right', bbox_to_anchor=(1.28, 1))
        plt.draw()        # Force matplotlib to update the canvas
        plt.pause(0.001)  # Flush the GUI event queue so the red line actually renders

    plt.show()            # Keep the final window open


if __name__ == '__main__':
    main()