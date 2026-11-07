
import math
import matplotlib.pyplot as plt
### A*

class AStarPlanner:
    def __init__(self, ox, oy, edge_model,
                 heuristic="euclidean",
                 show_animation=False,
                 min_x=0,
                 max_x=19,
                 min_y=0,
                 max_y=19):
        """
        ox: list of obstacle x coordinates (integers)
        oy: list of obstacle y coordinates (integers)
        edge_model: list of transitions [[dx, dy, cost], ...]
        min_x: world minimum x coordinate
        max_x: world maximum x coordinate
        min_y: world minimum y coordinate
        max_y: world maximum y coordinate
        heuristic: heuristic function to use
        show_animation: whether to show animation
        """
        self.edge_model = edge_model
        self.heuristic_selection = heuristic
        self.show_animation = show_animation

        # Bounds limits
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

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

        # Search statistics tracking
        expanded_vertices_count = 0
        peak_queue_size = len(Q)

        # Handles for dynamic live animation
        q_scatter = None
        closed_scatter = None
        current_node_scatter = None
        edge_lines = {}
        count = 0

        ### Line 6 ###
        while Q:

            # Update peak frontier size
            if len(Q) > peak_queue_size:
                peak_queue_size = len(Q)

            ### Line 7 ###
            v_key = min(Q, key=lambda v: Q[v].cost + self.calc_heuristic(Q[v]))
            v_next = Q.pop(v_key) # Replaces Line 15
            closed_set[v_key] = v_next

            expanded_vertices_count += 1

            if self.show_animation:
                if current_node_scatter is not None:
                    current_node_scatter.remove()
                current_node_scatter = plt.scatter(
                    v_next.x, v_next.y, c="yellow", edgecolors="black", s=70, zorder=7
                )

                if closed_scatter is not None:
                    closed_scatter.remove()
                cx = [node.x for node in closed_set.values()]
                cy = [node.y for node in closed_set.values()]
                closed_scatter = plt.scatter(
                    cx, cy, c="black", edgecolors="gray", linewidths=0.5, s=28, zorder=3
                )
            
            ### Line 8 ###
            if v_next.x == self.goal_node.x and v_next.y == self.goal_node.y:

                ### Line 9 ###
                self.goal_node.cost = v_next.cost
                self.goal_node.parent_index = v_next.parent_index
                break

            ### Line 10 ###
            for edge in self.edge_model:
                nx = v_next.x + edge[0]
                ny = v_next.y + edge[1]
                edge_vertex_pos = (nx, ny)

                ### Line 11 ###
                # A new vertex should belong to the open queue unless it has already been explored or is an obstacle
                # Case 1: Skip if the vertex is an obstacle or out of bounds
                if edge_vertex_pos in self.obstacle_map:
                    continue
                # Boundary check: reject nodes outside limits
                if not (self.min_x <= nx <= self.max_x and self.min_y <= ny <= self.max_y):
                    continue
                
                # Case 2: Skip if already explored
                if edge_vertex_pos in closed_set:
                    continue

                # we've confirmed it is in the open queue, but we need to check if its already initialized in Q
                if edge_vertex_pos not in Q:
                    u = self.Vertex(nx, ny, math.inf, -1)
                    Q[edge_vertex_pos] = u
                else:
                    u = Q[edge_vertex_pos]
                
                ### Line 12 ###
                cost_new = v_next.cost + edge[2]

                ### Line 13 ###
                if cost_new < u.cost:
                    u.cost = cost_new
                    u.parent_index = (v_next.x, v_next.y)

                    if self.show_animation:
                        if edge_vertex_pos in edge_lines:
                            edge_lines[edge_vertex_pos].remove()

                        (line,) = plt.plot(
                            [v_next.x, nx],
                            [v_next.y, ny],
                            color="black",
                            linewidth=0.8,
                            zorder=2,
                        )

                        edge_lines[edge_vertex_pos] = line

            # Live animation: update open set Q as cyan dots
            if self.show_animation:
                if q_scatter is not None:
                    q_scatter.remove()

                if Q:
                    qx = [node.x for node in Q.values()]
                    qy = [node.y for node in Q.values()]
                    q_scatter = plt.scatter(qx, qy, c='cyan', s=30, zorder=3)

                plt.pause(0.1)

        # Clean up remaining open set markers before plotting the final path
        # if self.show_animation and q_scatter is not None:
        #     q_scatter.remove()
            
        # Goal unreachable check
        if self.goal_node.parent_index == -1 and (start_node.x, start_node.y) != (
            self.goal_node.x,
            self.goal_node.y,
        ):
            print("\n--- Search Finished: No Path Found ---")
            print(f"Vertices expanded: {expanded_vertices_count}")
            print(f"Queue size when finished: {len(Q)}")
            return [], []

        rx, ry = self.calc_final_path(self.goal_node, closed_set)

        # Print final search statistics
        print("\n================ Search Statistics ================", flush=True)
        print(f"Vertices expanded        : {expanded_vertices_count}", flush=True)
        print(f"Queue size when finished : {len(Q)}", flush=True)
        print(f"Peak queue size          : {peak_queue_size}", flush=True)
        print(f"Solution steps (vertices): {len(rx)}", flush=True)
        print(f"Solution path cost (g)   : {self.goal_node.cost:.4f}", flush=True)
        print("===================================================\n", flush=True)

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

        # Render final path immediately onto the animation window
        if self.show_animation and rx:
            plt.plot(rx, ry, color='red', linewidth=2.5, zorder=5)
            plt.scatter(rx, ry, c='red', s=45, zorder=6)
            plt.draw()
            plt.pause(0.001)

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