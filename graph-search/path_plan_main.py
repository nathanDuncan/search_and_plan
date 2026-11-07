"""

Path planning

author: Nathan Duncan


"""

from matplotlib import axes
import math

import matplotlib.pyplot as plt

from environments import get_environment, get_obstacle_changes
from AStar import AStarPlanner
from LPAStar import LPAStarPlanner

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
    selected_env = '4a'
    flip_start_goal = False
    heuristic="euclidean"
    planner = "LPAStar"

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

    # Create batch scatter plots once during initialization
    free_scatter = ax.scatter([], [], c='grey', s=20, zorder=1, label="Free Nodes")
    obs_scatter = ax.scatter(
        [], [], c='white', edgecolors='black', linewidths=1.2, s=35, zorder=4, label="Obstacles"
    )

    def update_background(current_obstacles):
        free_pts = [
            (x, y)
            for x in range(min_x, max_x + 1)
            for y in range(min_y, max_y + 1)
            if (x, y) not in current_obstacles
        ]
        obs_pts = list(current_obstacles)

        # Fast hardware-accelerated update of points without recreating artists
        free_scatter.set_offsets(free_pts)
        obs_scatter.set_offsets(obs_pts)

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

    # Handles for drawing and redrawing the path
    path_line = None
    path_scatter = None

    def clear_path():
        nonlocal path_line, path_scatter
        if path_line is not None:
            path_line.remove()
            path_line = None
        if path_scatter is not None:
            path_scatter.remove()
            path_scatter = None
        plt.draw()

    def draw_path(rx, ry):
        nonlocal path_line, path_scatter
        clear_path()
        if rx and ry:
            (path_line,) = ax.plot(rx, ry, color='red', linewidth=2.5, zorder=8)
            path_scatter = ax.scatter(rx, ry, c='red', s=45, zorder=9)
            plt.draw()
            plt.pause(0.5)

    ### Helper to update background nodes upon environment transition ###
    def apply_environment_change(new_env_name):
        nonlocal obstacle_set

        # Clear the old red path
        clear_path()

        new_env = get_environment(new_env_name)
        new_ox, new_oy = [int(x) for x in new_env.ox], [int(y) for y in new_env.oy]
        new_obstacle_set = set(zip(new_ox, new_oy))

        added_obs, removed_obs = get_obstacle_changes(obstacle_set, new_obstacle_set)

        # Fast background update using offsets
        obstacle_set = new_obstacle_set
        update_background(obstacle_set)
        plt.draw()
        plt.pause(0.2)

        obstacle_set = new_obstacle_set

        return added_obs, removed_obs

    #####################################################
    ############# Solve Planning Problem ###############
    #####################################################
    # -------------- A* ---------------------------------
    if planner == 'AStar':
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

    # -------------- LPA* -------------------------------
    elif planner == 'LPAStar':
        lpa_star = LPAStarPlanner(
            sx, sy, gx, gy,
            ox=ox,
            oy=oy,
            edge_model=env.edge_model,
            show_animation=show_animation,
            min_x=min_x,
            max_x=max_x,
            min_y=min_y,
            max_y=max_y,
        )
        
        # Run LPA* (initial)
        lpa_star.initial()
        rx, ry = lpa_star.calc_final_path()
        draw_path(rx, ry)
        print("\n--- Expansions: ---", flush=True)
        plt.pause(20)

        ### Change to environment 4b ###
        print("\n--- Transitioning to Environment 4b ---", flush=True)
        added_obs, removed_obs = apply_environment_change('4b')
        ex1 = lpa_star.change(added_obs, removed_obs)
        new_rx, new_ry = lpa_star.calc_final_path()
        draw_path(new_rx, new_ry)
        print("\n--- Expansions: {} ---".format(ex1), flush=True)
        plt.pause(20)

        ### Change to nvironment 4c ###
        print("\n--- Transitioning to Environment 4c ---", flush=True)
        added_obs, removed_obs = apply_environment_change('4c')
        ex2 = lpa_star.change(added_obs, removed_obs)
        new_rx, new_ry = lpa_star.calc_final_path()
        draw_path(new_rx, new_ry)
        print("\n--- Expansions: {} ---".format(ex1), flush=True)
        plt.show()

    #####################################################
    ############# Plot Final Path #######################
    #####################################################
    
    if rx and ry:
        ax.plot(rx, ry, color='red', linewidth=2.5, zorder=5, label="Final Path")
        ax.scatter(rx, ry, c='red', s=45, zorder=6, label="Path Nodes")
        # ax.legend(loc='upper right', bbox_to_anchor=(1.28, 1))
        plt.draw()        # Force matplotlib to update the canvas
        plt.pause(0.001)  # Flush the GUI event queue so the red line actually renders

    plt.show()            # Keep the final window open


if __name__ == '__main__':
    main()