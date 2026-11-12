
import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from RRT import RRTPlanner
### RRT*

class RRTStarPlanner(RRTPlanner):

    class Vertex(RRTPlanner.Vertex):

        def __init__(self, x, y, cost=0.0):
            super().__init__(x, y)
            self.cost = cost

    def __init__(self, *args, max_iter=2500, log_interval=500, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_iter = max_iter
        self.log_interval = log_interval

    def planning(self, sx, sy, gx, gy):
        """
        RRT* path planning adapted from ELEC 844 (lecture8_handout slide 31).

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            stats: iterations, solution length, etc.
            rx: x position list of the final path
            ry: y position list of the final path
        """
        # Metrics initialization
        first_solution_iter = None
        first_solution_length = None
        length_history = {}
        
        ### Initialization ###
        self.goal = (gx, gy)
        goal_node = None

        ### Line 1 ###
        self.start = (sx, sy)
        start = self.Vertex(sx, sy)
        self.V = [start]

        ### Line 2 ###
        for i in range(2500):

            ### Line 3 ### 
            point_rand = self.Sample(self.eps)

            ### Line 4 ###
            v_nearest = min(self.V, key=lambda v: self.distance(v, point_rand))

            ### Line 5 ###
            point_new = self.Steer(v_nearest, point_rand, self.eta)

            ### Line 6 ###
            if self.IsEdgeValid(v_nearest, point_new):

                neighbours = self.Near(V, point_new)

                ### Line 7 ###
                v_best = v_nearest
                min_cost = v_nearest.cost + self.distance(v_nearest, point_new)

                for n in neighbours:
                    c = n.cost + self.distance(n, point_new)
                    if c < min_cost and self.IsEdgeValid(n, point_new):
                        v_best = n
                        min_cost = c

                ### Line 8 and 9 ###
                # Do I need an is edge valid here?
                v_new = self.Vertex(point_new[0], point_new[1])
                v_new.parent = v_best
                self.V.append(v_new)

                ### Line 10 ###
                for n in neighbours:
                    if n is v_best:
                        continue
                    c_new = v_new.cost + self.distance(v_new, n)
                    if c_new < n.cost and self.IsEdgeValid(v_new, n):
                        cost_diff = n.cost - c_new
                        n.parent = v_new
                        n.cost = c_new
                        self.shorten_branch_cost(n, cost_diff)

                # Keep goal_node cost synced if its ancestor branch was rewired
                if goal_node is not None and goal_node.parent is not None:
                    goal_node.cost = goal_node.parent.cost + self.distance(goal_node.parent, goal_node)

                # Check if v_new can connect to goal
                if self.distance(v_new, self.goal) <= self.eta and self.IsEdgeValid(v_new, self.goal):
                    total_goal_cost = v_new.cost + self.distance(v_new, self.goal)

                    if goal_node is None:
                        goal_node = self.Vertex(gx, gy, cost=total_goal_cost)
                        goal_node.parent = v_new

                        first_solution_iter = i
                        first_solution_length = goal_node.cost
                        
                    elif total_goal_cost < goal_node.cost:
                        goal_node.parent = v_new
                        goal_node.cost = total_goal_cost

            # Log solution length every 500 iterations
            if i % self.log_interval == 0:
                current_len = goal_node.cost if goal_node is not None else None
                length_history[i] = current_len

            # Animate search tree
            if self.show_animation:
                self.draw_graph(point_rand)

        # Backtrack path from best goal_node
        rx, ry = [], []
        path_length = 0.0

        if goal_node is not None:
        path_length = goal_node.cost
        curr = goal_node
        while curr is not None:
            rx.append(curr.x)
            ry.append(curr.y)
            curr = curr.parent
        rx.reverse()
        ry.reverse()

        stats = {
            'vertices_added': len(self.V),
            'total_iterations': self.max_iter,
            'path_length': path_length,
            'first_solution_iter': first_solution_iter,
            'first_solution_length': first_solution_length,
            'length_history': length_history,
        }

        if self.show_animation:
        self.draw_graph()
        if rx:
            plt.plot(rx, ry, '-r', linewidth=2.5, label='Path')
        plt.legend(loc='upper right')
        plt.show()

        return rx, ry, stats

    def Near(self, v_new):
        """ a k-nearest implementation
        """
        num_vertices = len(self.V)
        if num_vertices <= 1:
            return list(self.V)

        n = 2

        # aso guarentee (lecture8_handout slide 35)
        k = math.e * (1.0 + 1.0 / n) * math.log(num_vertices)
        k = math.ceil(k)

        # Sort vertices by distance to v_new and return the top k
        sorted_vertices = sorted(self.V, key=lambda v: self.distance(v, v_new))
        return sorted_vertices[:k]

    def shorten_branch_cost(self, parent_node, cost_reduction):
        """Update branch cost to reach after a reconnect"""
        for node in self.V:
            if node.parent is parent_node:
                node.cost -= cost_reduction
                self.shorten_branch_cost(node, cost_reduction)


    ##################################################
    ##### Visuals ######
    ##################################################
    def draw_graph(self, rnd=None):
        """Draws current tree, rectangular obstacles, start/goal, and random target."""
        plt.clf()
        ax = plt.gca()

        # Exit animation cleanly with escape key
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None]
        )

        # Plot all rectangular obstacles
        for (minx, maxx), (miny, maxy) in self.obstacles:
            width = maxx - minx
            height = maxy - miny
            rect = patches.Rectangle(
                (minx, miny), width, height,
                facecolor='black', edgecolor='none', alpha=0.8
            )
            ax.add_patch(rect)

        # Plot RRT tree branches
        for node in self.V:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g", linewidth=0.7)

        # Plot sampled point
        if rnd is not None:
            rx = rnd.x if hasattr(rnd, 'x') else rnd[0]
            ry = rnd.y if hasattr(rnd, 'y') else rnd[1]
            plt.plot(rx, ry, "^k", markersize=5)

        # Plot start and goal
        plt.plot(self.start[0], self.start[1], "xr", markersize=10, markeredgewidth=2, label="Start")
        plt.plot(self.goal[0], self.goal[1], "xb", markersize=10, markeredgewidth=2, label="Goal")

        plt.xlim(self.min_x, self.max_x)
        plt.ylim(self.min_y, self.max_y)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True)
        plt.pause(0.001)