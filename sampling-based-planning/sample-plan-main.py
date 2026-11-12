from RRT import RRTPlanner
from RRTConnect import RRTConnectPlanner
from continuous_environments import Environment

import numpy as np
import matplotlib.pyplot as plt

def print_summary_stats(name: str, values: list):
    """Calculates and prints min, max, Q1, median, Q3, and mean for a metric."""
    arr = np.array(values)
    
    val_min = np.min(arr)
    val_max = np.max(arr)
    q1 = np.percentile(arr, 25)
    median = np.median(arr)
    q3 = np.percentile(arr, 75)
    mean = np.mean(arr)
    
    print(f"\n--- {name} ---")
    print(f"  Min:          {val_min:8.2f}")
    print(f"  1st Quartile: {q1:8.2f}")
    print(f"  Median (Q2):  {median:8.2f}")
    print(f"  3rd Quartile: {q3:8.2f}")
    print(f"  Max:          {val_max:8.2f}")
    print(f"  Mean:         {mean:8.2f}")


def run_study(env_setting='1_l25', num_trials=50):
    env = Environment(env_setting)

    iterations_list = []
    vertices_list = []
    path_length_list = []
    successful_runs = 0

    print(f"Starting {num_trials} trials on environment '{env_setting}'...")

    for trial in range(1, num_trials + 1):
        planner = RRTPlanner(
            obstacles=env.obstacles,
            min_x=env.min_x,
            max_x=env.max_x,
            min_y=env.min_y,
            max_y=env.max_y,
            eta=2.5,
            eps=0.05,
            show_animation=False
        )

        rx, ry, stats = planner.planning(env.sx, env.sy, env.gx, env.gy)

        iterations_list.append(stats['total_iterations'])
        vertices_list.append(stats['vertices_added'])
        path_length_list.append(stats['path_length'])
        print(f"  [Trial {trial:02d}/{num_trials}] Finished | "
                f"Iter: {stats['total_iterations']} | Nodes: {stats['vertices_added']} | "
                f"Length: {stats['path_length']:.2f}")


    # Print stats
    print_summary_stats("Total Iterations", iterations_list)
    print_summary_stats("Vertices Added", vertices_list)
    print_summary_stats("Path Length (m)", path_length_list)

    # Plot histograms
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    # Total Iterations Histogram
    axes[0].hist(iterations_list, bins='auto', color='skyblue', edgecolor='black', alpha=0.7)
    axes[0].set_title("Total Iterations")
    axes[0].set_xlabel("Iterations")
    axes[0].set_ylabel("Frequency")
    axes[0].grid(axis='y', linestyle='--', alpha=0.7)

    # Vertices Added Histogram
    axes[1].hist(vertices_list, bins='auto', color='salmon', edgecolor='black', alpha=0.7)
    axes[1].set_title("Vertices Added to Tree")
    axes[1].set_xlabel("Number of Vertices")
    axes[1].set_ylabel("Frequency")
    axes[1].grid(axis='y', linestyle='--', alpha=0.7)

    # Path Length Histogram
    axes[2].hist(path_length_list, bins='auto', color='mediumseagreen', edgecolor='black', alpha=0.7)
    axes[2].set_title("Path Length")
    axes[2].set_xlabel("Length (m)")
    axes[2].set_ylabel("Frequency")
    axes[2].grid(axis='y', linestyle='--', alpha=0.7)

    plt.suptitle(f"RRT Performance Study ({env_setting}, N={num_trials})", fontsize=14)
    plt.tight_layout()
    plt.show()

def main():
    print(__file__ + " start!!")

    # run_study(env_setting='1_l25', num_trials=500)
    # return

    # Environment selection
    env = Environment('1_l25')

    # Planner holds search parameters (eta, eps, collision resolution)
    # planner = RRTPlanner(
    #     show_animation=True,
    #     obstacles=env.obstacles,
    #     min_x=env.min_x,
    #     max_x=env.max_x,
    #     min_y=env.min_y,
    #     max_y=env.max_y,
    #     eta=2.5,
    #     eps=0.01
    # )

    planner = RRTConnectPlanner(
        show_animation=True,
        obstacles=env.obstacles,
        min_x=env.min_x,
        max_x=env.max_x,
        min_y=env.min_y,
        max_y=env.max_y,
        eta=2.5
    )

    # Execute
    rx, ry, stats = planner.planning(env.sx, env.sy, env.gx, env.gy)

    print("Planning Complete!")
    print(f"  Total Iterations:           {stats['total_iterations']}")
    print(f"  Vertices in Tree:           {stats['vertices_added']}")
    print(f"  Path Length:                {stats['path_length']:.2f} m")


if __name__ == '__main__':
    main()