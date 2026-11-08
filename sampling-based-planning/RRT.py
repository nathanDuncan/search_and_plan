
import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
### RRT

class RRTPlanner:

    class Vertex:
        def __init__(self, x, y):
            self.x = x
            self.y = y 
            
            self.parent = None

        def __str__(self):
            return str(self.x) + "," + str(self.y)

    def __init__(self, obstacles=None,
                 show_animation=False,
                 min_x=0,
                 max_x=100,
                 min_y=0,
                 max_y=100,
                 eta=2.0,
                 eps=0.05,
                 collision_step_size=0.5):
        """
        obstacles: list of tuples [((minx, maxx), (miny, maxy)), ...]
        min_x: world minimum x coordinate
        max_x: world maximum x coordinate
        min_y: world minimum y coordinate
        max_y: world maximum y coordinate
        eta: maximum step distance for steering
        eps: goal sampling probability (epsilon)
        collision_step_size: resolution step along edges for collision checks
        """
        self.show_animation = show_animation

        # Bounds limits
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

        # Parameters
        self.eta = eta
        self.eps = eps
        self.collision_step_size = collision_step_size

        # Obstacle representation: list of ((minx, maxx), (miny, maxy))
        self.obstacles = obstacles if obstacles is not None else []

        # Vertex container
        self.V = []
        self.goal = None

    def planning(self, sx, sy, gx, gy):
        """
        RRT path planning adapted from ELEC 844 (lecture7_handout slide 37).

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
        self.goal = (gx, gy)
        goal_node = None

        ### Line 1 ###
        self.start = (sx, sy)
        start = self.Vertex(sx, sy)
        self.V = [start]

        ### Line 2 ###
        """ Edges are stored in vertex struct. """

        ### Line 3 ###
        while goal_node is None:
            total_iterations += 1

            ### Line 4 ###
            point_rand = self.Sample(self.eps)

            ### Line 5 ###
            v_nearest = min(self.V, key=lambda v: self.distance(v, point_rand))

            ### Line 6 ###
            point_new = self.Steer(v_nearest, point_rand, self.eta)

            ### Line 7 ###
            if self.IsEdgeValid(v_nearest, point_new):

                ### Line 8 and 9 ###
                v_new = self.Vertex(point_new[0], point_new[1])
                v_new.parent = v_nearest
                self.V.append(v_new)

            ### (Line 3 again) Check if the goal region is reached ###
            if self.distance(v_new, self.goal) <= self.eta:
                goal_node = self.Vertex(gx, gy)
                goal_node.parent = v_new
                self.V.append(goal_node)
                break

            # Animate search tree
            if self.show_animation:
                self.draw_graph(point_rand)

        ### Line 10 ###
        ### Generate path by tracing back from goal_node ###
        # Backtrack path
        rx, ry = [], []
        path_length = 0.0

        if goal_node is not None:
            curr = goal_node
            while curr is not None:
                rx.append(curr.x)
                ry.append(curr.y)
                if curr.parent is not None:
                    path_length += self.distance(curr, curr.parent)
                curr = curr.parent

            rx.reverse()
            ry.reverse()

        # Build stats payload
        stats = {
            'vertices_added': len(self.V),
            'total_iterations': total_iterations,
            'path_length': path_length
        }

        if self.show_animation:
            self.draw_graph()
            plt.plot(rx, ry, '-r', linewidth=2.5, label="Path")
            plt.legend(loc="upper right")
            plt.show()

        return rx, ry, stats

    def Sample(self, eps):
        """ Sample function (lecture7_handout slide 38) """
        ### Line 1 ###
        p = random.uniform(0, 1)

        ### Line 2 ###
        if p < self.eps:

            ### Line 3 ###
            point_rand = self.goal

        ### Line 4 ###
        else:

            ### Line 5 ###
            while True:

                ### Line 6 ###
                point_rand = (random.uniform(self.min_x, self.max_x),
                              random.uniform(self.min_y, self.max_y))

                ### Line 7 ###
                if self.isFreeSpace(point_rand):
                    break

        ### Line 8 ###
        return point_rand

    def Steer(self, v_nearest, point_rand, eta):
        """
        From (lecture7_handout slide 38)
        Steer(v_nearest, x_rand, eta) := argmin_{y in X} { distance(y, x_rand) | distance(v_nearest, y) <= eta }
        """
        d = self.distance(v_nearest, point_rand)

        # If already within eta distance, reach the target point directly
        if d <= eta:
            px = point_rand.x if hasattr(point_rand, 'x') else point_rand[0]
            py = point_rand.y if hasattr(point_rand, 'y') else point_rand[1]
            return (px, py)

        # Otherwise, move eta units along the line connecting v_nearest to point_rand
        px = point_rand.x if hasattr(point_rand, 'x') else point_rand[0]
        py = point_rand.y if hasattr(point_rand, 'y') else point_rand[1]

        theta = math.atan2(py - v_nearest.y, px - v_nearest.x)
        new_x = v_nearest.x + eta * math.cos(theta)
        new_y = v_nearest.y + eta * math.sin(theta)

        return (new_x, new_y)

    def isFreeSpace(self, point):
        """ works with tuples or vertex struct """
        px = point.x if hasattr(point, 'x') else point[0]
        py = point.y if hasattr(point, 'y') else point[1]

        for (minx, maxx), (miny, maxy) in self.obstacles:
            if minx <= px <= maxx and miny <= py <= maxy:
                return False

        return True

    def IsEdgeValid(self, v_from, v_to):
        """
        Check edge at discrete steps for collision 
        """
        x1 = v_from.x if hasattr(v_from, 'x') else v_from[0]
        y1 = v_from.y if hasattr(v_from, 'y') else v_from[1]
        x2 = v_to.x if hasattr(v_to, 'x') else v_to[0]
        y2 = v_to.y if hasattr(v_to, 'y') else v_to[1]

        dist = math.hypot(x2 - x1, y2 - y1)
        if dist == 0:
            return self.isFreeSpace((x1, y1))

        # Number of samples along the edge
        num_steps = max(int(math.ceil(dist / self.collision_step_size)), 1)

        for step in range(num_steps + 1):
            t = step / num_steps
            interp_x = x1 + t * (x2 - x1)
            interp_y = y1 + t * (y2 - y1)

            if not self.isFreeSpace((interp_x, interp_y)):
                return False

        return True

    @staticmethod
    def distance(p1, p2):
        """ works with tuples or vertex struct """
        x1 = p1.x if hasattr(p1, 'x') else p1[0]
        y1 = p1.y if hasattr(p1, 'y') else p1[1]
        x2 = p2.x if hasattr(p2, 'x') else p2[0]
        y2 = p2.y if hasattr(p2, 'y') else p2[1]

        return math.hypot(x2 - x1, y2 - y1)


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