
import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from RRT import RRTPlanner
### RRTConnect

class RRTConnectPlanner(RRTPlanner):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Store both trees
        self.V_a = []
        self.V_b = []

    def planning(self, sx, sy, gx, gy):
        """
        RRT path planning adapted from ELEC 844 (lecture8_handout slide 13).

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        # Metrics initialization
        total_iterations = 0

        ### Initialization ###
        connected = False
        v_conn_a = None
        v_conn_b = None

        self.start = (sx, sy)
        self.goal = (gx, gy)

        start_node = self.Vertex(sx, sy)
        goal_node = self.Vertex(gx, gy)

        ### Line 1 ###
        self.V_a = [start_node]

        ### Line 2 ###
        self.V_b = [goal_node]

        ### Line 3 ###
        """Terminal condition: Va \cup Vb \neq \empty"""
        while not connected:
            total_iterations += 1

            ### Line 4-6 ###
            point_rand = self.Sample()

            ### Line 7 ###
            v_nearest = min(self.V_a, key=lambda v: self.distance(v, point_rand))

            ### Line 8 ###
            point_new = self.Steer(v_nearest, point_rand, self.eta)

            ### Line 9 ### 
            if self.IsEdgeValid(v_nearest, point_new):

                ### Line 10-11 ###
                v_new = self.Vertex(point_new[0], point_new[1])
                v_new.parent = v_nearest
                self.V_a.append(v_new)

                ### Line 12 ###
                connected, v_conn_b = self.Connect(self.V_b, v_new)
                if connected:
                    v_conn_a = v_new
                    break

            ### Line 13-14 ###
            self.V_a, self.V_b = self.V_b, self.V_a

            if self.show_animation:
                self.draw_graph(point_rand)

        # Sum both trees
        path_a, dist_a = self.trace_branch(v_conn_a, 0.0)
        path_b, dist_b = self.trace_branch(v_conn_b, 0.0)

        full_path = path_a[::-1] + path_b
        rx = [p[0] for p in full_path]
        ry = [p[1] for p in full_path]

        path_length = dist_a + dist_b + self.distance(v_conn_a, v_conn_b)

        # Build stats payload
        stats = {
            "vertices_added": len(self.V_a) + len(self.V_b),
            "total_iterations": total_iterations,
            "path_length": path_length,
        }

        # Final visualization
        if self.show_animation:
            self.draw_graph()
            plt.plot(rx, ry, "-r", linewidth=2.5, label="Final Path")
            plt.legend(loc="upper right")
            plt.show()

        return rx, ry, stats

    def Connect(self, V_b, v_connect):
        """
        From (lecture8_handout slide 13)
        """
        ### Line 1 ###
        v_nearest_b = min(V_b, key=lambda v: self.distance(v, v_connect))

        ### Line 2 ###
        while True:

            ### Line 3 ### 
            point_step = self.Steer(v_nearest_b, v_connect, self.eta)

            ### Line 4 ###
            if self.IsEdgeValid(v_nearest_b, point_step):

                ### Line 5-6 ###
                v_step = self.Vertex(point_step[0], point_step[1])
                v_step.parent = v_nearest_b
                V_b.append(v_step)

                ### Line 7 ###
                v_nearest_b = v_step

                if self.show_animation:
                    self.draw_graph(v_connect)

                ### Line 8: Check if target node has been reached ###
                if (
                    self.distance(v_nearest_b, v_connect) <= self.eta
                    and self.IsEdgeValid(v_nearest_b, v_connect)
                ):
                    return True, v_nearest_b

            else:
                ### Line 9: Hit an obstacle (trapped) ###
                return False, None
            

    # ------------ Little helper functions ----------------------------------------------------------------
    def trace_branch(self, vert, distance):
        path = []
        curr = vert
        while curr is not None:
            path.append((curr.x, curr.y))
            if curr.parent is not None:
                distance += self.distance(curr, curr.parent)
            curr = curr.parent
        return path, distance


    ##################################################
    ##### Visuals ######
    ##################################################
    def draw_graph(self, rnd=None):
        """Draws both search trees, obstacles, start/goal, and random target."""
        plt.clf()
        ax = plt.gca()

        # Exit animation cleanly with escape key
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )

        # Plot all rectangular obstacles
        for (minx, maxx), (miny, maxy) in self.obstacles:
            width = maxx - minx
            height = maxy - miny
            rect = patches.Rectangle(
                (minx, miny),
                width,
                height,
                facecolor="black",
                edgecolor="none",
                alpha=0.8,
            )
            ax.add_patch(rect)

        # Plot Tree A branches (Green)
        for node in self.V_a:
            if node.parent:
                plt.plot(
                    [node.x, node.parent.x],
                    [node.y, node.parent.y],
                    "-g",
                    linewidth=0.7,
                )

        # Plot Tree B branches (Royal Blue)
        for node in self.V_b:
            if node.parent:
                plt.plot(
                    [node.x, node.parent.x],
                    [node.y, node.parent.y],
                    "-b",
                    linewidth=0.7,
                )

        # Plot sampled target point
        if rnd is not None:
            rx = rnd.x if hasattr(rnd, "x") else rnd[0]
            ry = rnd.y if hasattr(rnd, "y") else rnd[1]
            plt.plot(rx, ry, "^k", markersize=5)

        # Plot start and goal
        plt.plot(
            self.start[0],
            self.start[1],
            "xr",
            markersize=10,
            markeredgewidth=2,
            label="Start",
        )
        plt.plot(
            self.goal[0],
            self.goal[1],
            "xb",
            markersize=10,
            markeredgewidth=2,
            label="Goal",
        )

        plt.xlim(self.min_x, self.max_x)
        plt.ylim(self.min_y, self.max_y)
        plt.gca().set_aspect("equal", adjustable="box")
        plt.grid(True)
        plt.pause(1)