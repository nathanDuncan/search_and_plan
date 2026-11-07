
import math
import matplotlib.pyplot as plt
### LPA*

class LPAStarPlanner:
    def __init__(self, 
                 sx, sy, gx, gy,
                 ox, oy, edge_model,
                 heuristic="euclidean",
                 show_animation=False,
                 min_x=0,
                 max_x=19,
                 min_y=0,
                 max_y=19):
        """
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
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
        self.start = (int(round(sx)), int(round(sy)))
        self.goal = (int(round(gx)), int(round(gy)))

        self.edge_model = edge_model
        self.heuristic_selection = heuristic
        self.show_animation = show_animation

        # Bounds limits
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

        # Set of obstacle (x, y)
        self.obstacle_map = set(zip(ox, oy))

        # LPA* state maps: (x, y) -> value
        self.cost = {}      # g-values
        self.look = {}      # rhs-values
        self.parent = {}    # (x, y) -> parent (x, y)
        self.Q = {}         # (x, y) -> (key1, key2)

        # Matplotlib visualization handles
        self.edge_lines = {}
        self.q_scatter = None
        self.v_scatter = None

    # --- Graph Helpers ---
    def in_bounds(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def is_free(self, pos):
        return self.in_bounds(pos[0], pos[1]) and (pos not in self.obstacle_map)

    def get_neighbors(self, pos):
        """Returns list of (neighbor_pos, edge_cost) for valid adjacent nodes."""
        neighbors = []
        for edge in self.edge_model:
            nxt = (pos[0] + edge[0], pos[1] + edge[1])
            if self.is_free(nxt):
                neighbors.append((nxt, edge[2]))
        return neighbors

    def get_cost(self, u):
        return self.cost.get(u, math.inf)

    def get_look(self, u):
        return self.look.get(u, math.inf)

    def calc_heuristic(self, pos):
        # ii. Euclidian Distance Heuristic
        return math.hypot(pos[0] - self.goal[0], pos[1] - self.goal[1])

    def calc_key(self, v):
        min_c = min(self.get_cost(v), self.get_look(v))
        return (min_c + self.calc_heuristic(v), min_c)

    def top_key(self):
        if not self.Q:
            return (math.inf, math.inf)
        return min(self.Q.values())

    # --------------------------------------------------------
    # ------------------- Initialization ---------------------
    # --------------------------------------------------------
    def initial(self):
        """
        Initialize the LPA* search adapted from lecture pseudo code (lecture 5, slide 55).

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        self.cost.clear()
        self.look.clear()
        self.parent.clear()
        self.Q.clear()

        ### Line 1-2 ###
        """
        Like A* we will grow the open queue dynamically so we skip these steps for all nodes except the start
        """
        self.cost[self.start] = math.inf

        ### Line 3 ###
        self.look[self.start] = 0.0

        self.parent[self.start] = -1

        ### Line 4 ###
        self.Q[self.start] = self.calc_key(self.start)

        ### Line 5 ###
        self.search()

    # --------------------------------------------------------
    # ------------------- Search -----------------------------
    # --------------------------------------------------------
    def search(self):
        """
        LPA* path search adapted from lecture pseudo code (lecture 5, slide 57)

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        # Performance tracking
        expansions = 0

        ### Line 1 ###
        while (self.top_key() < self.calc_key(self.goal)) or (
            self.get_cost(self.goal) != self.get_look(self.goal)
        ):

            ### Line 2 ###
            vn = min(self.Q, key=lambda v: self.Q[v])

            ### Line 3 ###
            del self.Q[vn]

            expansions += 1

            if self.show_animation:
                if self.v_scatter:
                    self.v_scatter.remove()
                    self.v_scatter = None
                self.v_scatter = plt.scatter(
                    vn[0], vn[1], c="yellow", edgecolors="black", s=70, zorder=7
                )

            ### Line 4 ###
            if self.get_cost(vn) > self.get_look(vn):

                ### Line 5 ###
                self.cost[vn] = self.get_look(vn)

            ### Line 6 ###
            else:
                ### Line 7 ###
                self.cost[vn] = math.inf
                ### Line 8 ###
                min_val = math.inf
                best_parent = None
                for u, step_cost in self.get_neighbors(vn):
                    cand = self.get_cost(u) + step_cost
                    if cand < min_val:
                        min_val = cand
                        best_parent = u
                self.look[vn] = min_val
                self.parent[vn] = best_parent

                ### Line 9 ### 
                if self.get_cost(vn) != self.get_look(vn):
                    ### Line 10 ###
                    self.Q[vn] = self.calc_key(vn)
                    
            ### Line 11 ###
            for w, _ in self.get_neighbors(vn):
                ### Line 12 ###
                self.updateVertex(w)

            # Refresh animation
            if self.show_animation:
                if self.q_scatter:
                    self.q_scatter.remove()
                    self.q_scatter = None
                if self.Q:
                    qx = [p[0] for p in self.Q]
                    qy = [p[1] for p in self.Q]
                    self.q_scatter = plt.scatter(
                        qx, qy, c="cyan", edgecolors="blue", linewidths=0.5, s=35, zorder=4
                    )
                plt.pause(0.01)
            
        if self.show_animation and self.v_scatter:
            self.v_scatter.remove()
            self.v_scatter = None

        return expansions

    # ------------------------------------------------
    # -------------- Update Vertex -------------------
    # ------------------------------------------------
    def updateVertex(self, w):
        """
        adapted from lecture pseudo code (lecture 5, slide 57)
        Updated vertex u by removing it from the open queue if it is present
        and adding it to the open queue with its new key
        """
        best_parent = None
        min_val = math.inf

        ### Line 1 ###
        if w != self.start:
            # look[w] = min_{(u, w) in E} { cost[u] + c(u, w) }
            min_val = math.inf
            best_parent = None
            for u, step_cost in self.get_neighbors(w):
                cand = self.get_cost(u) + step_cost
                if cand < min_val:
                    min_val = cand
                    best_parent = u

            self.look[w] = min_val
            self.parent[w] = best_parent

        ### Line 2 ###
        if w in self.Q:
            ### Line 3 ###
            del self.Q[w]
        
        ### Line 4 ###
        if self.get_cost(w) != self.get_look(w):
            ### Line 5 ###
            self.Q[w] = self.calc_key(w)

        # Display
        if self.show_animation:
            if w in self.edge_lines:
                self.edge_lines[w].remove()
                del self.edge_lines[w]
            if best_parent is not None and min_val < math.inf:
                (line,) = plt.plot(
                    [best_parent[0], w[0]],
                    [best_parent[1], w[1]],
                    color="black",
                    linewidth=0.8,
                    zorder=2,
                )
                self.edge_lines[w] = line

    # ---------------------------------------
    # ------------ Change -------------------
    # ---------------------------------------
    def change(self, added_obs=None, removed_obs=None):
        """
        Adapts the search tree to environment changes (lecture 5, slide 55).
        Calls UpdateVertex on all vertices affected by changed edges, then resumes search().
        """
        if added_obs is None:
            added_obs = set()
        if removed_obs is None:
            removed_obs = set()

        affected_vertices = set() # Vertices of E_changed

        # 1. Process newly added obstacles
        for obs in added_obs:
            self.obstacle_map.add(obs)
            affected_vertices.add(obs)

            # A. Remove incoming edge pointing directly to this now-blocked node
            if self.show_animation and obs in self.edge_lines:
                self.edge_lines[obs].remove()
                del self.edge_lines[obs]

            # Invalidate parent and cost
            self.parent[obs] = None
            self.cost[obs] = math.inf
            self.look[obs] = math.inf

            # B. Check all neighbors: remove any outgoing edges that originated from this obstacle
            for edge in self.edge_model:
                nbr = (obs[0] + edge[0], obs[1] + edge[1])
                if self.in_bounds(nbr[0], nbr[1]):
                    affected_vertices.add(nbr)
                    # If this neighbor had the obstacle as its parent, erase its edge line
                    if self.parent.get(nbr) == obs:
                        self.parent[nbr] = None
                        self.look[nbr] = math.inf
                        if self.show_animation and nbr in self.edge_lines:
                            self.edge_lines[nbr].remove()
                            del self.edge_lines[nbr]

        # 2. Process removed obstacles (freed nodes)
        for obs in removed_obs:
            self.obstacle_map.discard(obs)
            affected_vertices.add(obs)
            for edge in self.edge_model:
                nbr = (obs[0] + edge[0], obs[1] + edge[1])
                if self.in_bounds(nbr[0], nbr[1]):
                    affected_vertices.add(nbr)

        ### Line 1 ###
        for w in affected_vertices:
            ### Line 2 ###
            self.updateVertex(w)

        ### Line 3 ###
        expansions = self.search()
        return expansions

        # # Goal unreachable check
        # if self.goal_node.parent_index == -1 and (start_node.x, start_node.y) != (
        #     self.goal_node.x,
        #     self.goal_node.y,
        # ):
        #     print("\n--- Search Finished: No Path Found ---")
        #     print(f"Vertices expanded: {expanded_vertices_count}")
        #     print(f"Queue size when finished: {len(Q)}")
        #     return [], []

        # rx, ry = self.calc_final_path(self.goal_node, closed_set)

        # # Print final search statistics
        # print("\n================ Search Statistics ================", flush=True)
        # print(f"Vertices expanded        : {expanded_vertices_count}", flush=True)
        # print(f"Queue size when finished : {len(Q)}", flush=True)
        # print(f"Peak queue size          : {peak_queue_size}", flush=True)
        # print(f"Solution steps (vertices): {len(rx)}", flush=True)
        # print(f"Solution path cost (g)   : {self.goal_node.cost:.4f}", flush=True)
        # print("===================================================\n", flush=True)

        # return rx, ry

    def calc_final_path(self):
        if self.get_cost(self.goal) == math.inf:
            print("LPA*: No valid path found to goal.", flush=True)
            return [], []

        rx = [self.goal[0]]
        ry = [self.goal[1]]
        curr = self.goal

        while curr != self.start:
            curr = self.parent.get(curr, None)
            if curr is None or curr == -1:
                return [], []
            rx.append(curr[0])
            ry.append(curr[1])

        rx.reverse()
        ry.reverse()

        return rx, ry