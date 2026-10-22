
import math
import matplotlib.pyplot as plt
### A*

class AStarPlanner:
    def __init__(self, ox, oy, edge_model, heuristic="euclidean", show_animation=False):
        """
        ox: list of obstacle x coordinates (integers)
        oy: list of obstacle y coordinates (integers)
        edge_model: list of transitions [[dx, dy, cost], ...]
        """
        self.edge_model = edge_model
        self.heuristic_selection = heuristic
        self.show_animation = show_animation

        # Set of obstacle (x, y) coordinates for O(1) lookups
        self.obstacle_map = set(zip(ox, oy))

    class Vertex:
        def __init__(self, x, y, cost, parent_index):
            self.x = int(round(x))
            self.y = int(round(y))
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
        start_node = self.Vertex(sx, sy, 0.0, -1)
        self.goal_node = self.Vertex(gx, gy, 0.0, -1)

        Q = dict()
        closed_set = dict() # Record of visited vertices

        ### Line 1 ###
        start_node.cost = 0.0

        ### Line 2 ###
        Q[(start_node.x, start_node.y)]= start_node

        ### Line 3-5 ###
        """
        In the algorithm discussed in class, the open queue is defined as ALL unexplored verticies.
        In this implementation, the open set is generated dyanmically as the algorithm expands outward.
        In essence, we only search the frontier, not the whole unexplored workspace.
        Therefore, Lines 4 and 5 will be modified and referenced later.
        """

        ### Line 6 ###
        while Q:

            ### Line 7 ###
            v_key = min(Q, key=lambda v: Q[v].cost + self.calc_heuristic(Q[v]))
            v_next = Q.pop(v_key) # Replaces Line 15
            closed_set[v_key] = v_next
            
            ### Line 8 ###
            if v_next.x == self.goal_node.x and v_next.y == self.goal_node.y:

                ### Line 9 ###
                self.goal_node.cost = v_next.cost
                self.goal_node.parent_index = v_next.parent_index
                break

            ### Line 10 ###
            for edge in self.edge_model:
                edge_vertex_pos = (v_next.x + edge[0],
                                   v_next.y + edge[1])

                ### Line 11 ###
                # A new vertex should belong to the open queue unless it has already been explored or is an obstacle
                # Case 1: Skip if the vertex is an obstacle 
                if edge_vertex_pos in self.obstacle_map:
                    continue
                
                # Case 2: Skip if already explored
                if edge_vertex_pos in closed_set:
                    continue

                # we've confirmed it is in the open queue, but we need to check if its already initialized in Q
                if edge_vertex_pos not in Q:
                    u = self.Vertex(edge_vertex_pos[0], edge_vertex_pos[1], math.inf, -1)
                    Q[edge_vertex_pos] = u
                else:
                    u = Q[edge_vertex_pos]
                
                ### Line 12 ###
                cost_new = v_next.cost + edge[2]

                ### Line 13 ###
                if cost_new < u.cost:
                    u.cost = cost_new
                    u.parent_index= (v_next.x, v_next.y)

            if (v_next.x, v_next.y) == (self.goal_node.x, self.goal_node.y):
                print("Goal reached")
                break
            


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

        rx, ry = self.calc_final_path(self.goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        rx = [goal_node.x]
        ry = [goal_node.y]
        parent_index = goal_node.parent_index

        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        rx.reverse()
        ry.reverse()
        return rx, ry

    def calc_heuristic(self, v):
        # i. Zero Heuristic
        if self.heuristic_selection == "zero":
            h_hat = 0
        # ii. Euclidian Distance Heuristic
        elif self.heuristic_selection == "euclidean":
            h_hat = math.hypot(v.x - self.goal_node.x, v.y - self.goal_node.y)
        # iii. Manhattan Distance Heuristic
        elif self.heuristic_selection == "manhattan":
            h_hat = abs(v.x - self.goal_node.x) + abs(v.y - self.goal_node.y)
        # iv. Inflated Heuristic
        elif self.heuristic_selection == "inflated":
            alpha = 100
            h_hat = alpha * math.hypot(v.x - self.goal_node.x, v.y - self.goal_node.y)
        else:
            raise ValueError("Invalid heuristic selection")
        return h_hat