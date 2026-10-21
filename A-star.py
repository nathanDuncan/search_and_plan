
import math
import matplotlib.pyplot as plt
### A*

class AStarPlanner():
    def __init__(self, ox, oy, resolution, edge_model, heuristic, show_animation):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.edge_model = edge_model
        self.heuristic_selection = heuristic
        self.resolution = resolution
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

        self.show_animation = show_animation

    class Vertex:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search adapted from ELEC 844 lecture slides.

        For ease of implementation the environment is not structured as a true graph.

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        
        ### Initialization ###
        start_node = self.Vertex(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        self.goal_node = self.Vertex(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        Q = dict()

        ### Line 1 ###
        self.start_node.cost = 0

        ### Line 2-5 ###
        """
        In the algorithm discussed in class, the open queue is defined as ALL unexplored verticies.
        In this implementation, the open set is generated dyanmically as the algorithm expands outward.
        In essence, we only search the frontier, not the whole unexplored workspace.
        Therefore, Lines 4 and 5 will be modified and referenced later.
        """

        ### Line 6 ###
        while not Q.empty:

            ### Line 7 ###
            v_next = min(
                Q,
                key=lambda v: Q[v].cost + self.calc_heuristic(Q[v]))
            
            ### Line 8-9 ###
            """
            Implicitly defined in all four available heuristics
            """

            ### Line 10 ###
            for edge in self.edge_model:
                u = self.Vertex(edge[0],
                              edge[1], math.inf, -1)

                ### Line 11 ###
                # a new node should belong to the open queue unless it has already been explored or is an obstacle

                ### Line 12 ###
                cost_new = v_next.cost + edge[2]

                ### Line 13 ###
                if cost_new < u.cost:
                    u.cost = cost_new
            
            ### Line 15 ###
            del Q[v_next]
            


        #  = dict(), dict()
        # open_set[self.calc_grid_index(start_node)] = start_node

        # while True:
        #     if len(open_set) == 0:
        #         print("Open set is empty..")
        #         break

        #     c_id = min(
        #         open_set,
        #         key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
        #                                                              open_set[
        #                                                                  o]))
        #     current = open_set[c_id]

        #     # show graph
        #     if self.show_animation:  # pragma: no cover
        #         plt.plot(self.calc_grid_position(current.x, self.min_x),
        #                  self.calc_grid_position(current.y, self.min_y), "xc")
        #         # for stopping simulation with the esc key.
        #         plt.gcf().canvas.mpl_connect('key_release_event',
        #                                      lambda event: [exit(
        #                                          0) if event.key == 'escape' else None])
        #         if len(closed_set.keys()) % 10 == 0:
        #             plt.pause(0.001)

        #     if current.x == self.goal_node.x and current.y == self.goal_node.y:
        #         print("Find goal")
        #         self.goal_node.parent_index = current.parent_index
        #         self.goal_node.cost = current.cost
        #         break

        #     # Remove the item from the open set
        #     del open_set[c_id]

        #     # Add it to the closed set
        #     closed_set[c_id] = current

        #     # expand_grid search grid based on motion model
        #     for i, _ in enumerate(self.motion):
        #         node = self.Node(current.x + self.motion[i][0],
        #                          current.y + self.motion[i][1],
        #                          current.cost + self.motion[i][2], c_id)
        #         n_id = self.calc_grid_index(node)

        #         # If the node is not safe, do nothing
        #         if not self.verify_node(node):
        #             continue

        #         if n_id in closed_set:
        #             continue

        #         if n_id not in open_set:
        #             open_set[n_id] = node  # discovered a new node
        #         else:
        #             if open_set[n_id].cost > node.cost:
        #                 # This path is the best until now. record it
        #                 open_set[n_id] = node

        # rx, ry = self.calc_final_path(self.goal_node, closed_set)

        # return rx, ry

    def calc_heuristic(self, v):
        # i. Zero Heuristic
        if self.heuristic_selection == 1:
            h_hat = 0
        # ii. Euclidian Distance Heuristic
        elif self.heuristic_selection == 2:
            h_hat = math.hypot(v.x - self.goal_node.x, v.y - self.goal_node.y)
        # iii. Manhattan Distance Heuristic
        elif self.heuristic_selection == 3:
            h_hat = abs(v.x - self.goal_node.x) + abs(v.y - self.goal_node.y)
        # iv. Inflated Heuristic
        elif self.heuristic_selection == 4:
            alpha = 100
            h_hat = alpha * math.hypot(v.x - self.goal_node.x, v.y - self.goal_node.y)
        else:
            raise ValueError("Invalid heuristic selection")
        return h_hat