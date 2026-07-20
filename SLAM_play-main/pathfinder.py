import heapq
import math
import numpy as np
from debug import log_pathfinder

class Pathfinder:
    """
    D* Lite Pathfinder.
    Unlike A* which plans from Start -> Goal, D* Lite plans Backward (Goal -> Start).
    It caches its math. When a new wall appears, it doesn't start over; it simply
    "patches" the math around the new wall and replans in microseconds.
    """

    def __init__(self):
        # D* Lite State variables
        self.U = []       # Priority Queue (like open_list in A*). Holds cells that need their math checked.
        self.g = {}       # Actual cost from this cell to the Goal. (Counts DOWN to the goal).
        self.rhs = {}     # Right-Hand Side: A one-step lookahead. "Based on my neighbors, what SHOULD my g score be?"
        self.km = 0       # Heuristic accumulator. If the robot moves, we add the distance to km instead of recalculating the whole queue.
        self.prev_map = None  # A snapshot of the map from the last frame, used to find newly discovered walls.
        self.goal = None
        self.start = None
        self.map_grid = None

    def _key(self, s):
        """
        Calculates the priority key for a cell. The queue is sorted by this key.
        Key = (k1, k2)
        k1 = min(g, rhs) + h + km  -> Acts like f-score in A* (estimates total path cost).
        k2 = min(g, rhs)           -> Tie-breaker. If two cells have the same k1, pick the one with lower actual cost.
        """
        g_val = self.g.get(s, float('inf'))
        rhs_val = self.rhs.get(s, float('inf'))
        h_val = self._calculate_heuristic(self.start, s)
        return (min(g_val, rhs_val) + h_val + self.km, min(g_val, rhs_val))

    def _is_free(self, cell):
        """Check if a cell is traversable (definitely free space)."""
        if not (0 <= cell[0] < self.map_grid.shape[0] and 0 <= cell[1] < self.map_grid.shape[1]):
            return False # Out of bounds
        # In your SLAM, >= -0.8 is obstacle/unknown. < -0.8 is definitely free.
        return self.map_grid[cell] < -0.8

    def _cost(self, u, v):
        """Cost to move from cell u to cell v."""
        if not self._is_free(v): 
            return float('inf') # Can't walk through walls
            
        # Corner cutting prevention: If moving diagonally, check the two adjacent cells.
        # If either is a wall, we can't cut diagonally!
        if u[0] != v[0] and u[1] != v[1]:
            cell1 = (u[0] + (v[0]-u[0]), u[1])
            cell2 = (u[0], u[1] + (v[1]-u[1]))
            if not self._is_free(cell1) or not self._is_free(cell2):
                return float('inf')
                
        # Diagonal movement costs 1.414, straight costs 1.0
        return 1.414 if (u[0] != v[0] and u[1] != v[1]) else 1.0

    def _get_neighbors(self, s):
        """Return 8-way neighbors for a given cell."""
        neighbors = []
        directions = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
        for dx, dy in directions:
            neighbors.append((s[0] + dx, s[1] + dy))
        return neighbors

    def _update_vertex(self, s):
        """
        Recalculate the rhs (lookahead) value of a cell and update the priority queue.
        This is the "Patching" function. When a wall appears, we call this on the affected cells.
        """
        if s != self.goal:
            # Look at all neighbors and find the cheapest path to the goal from this cell
            min_rhs = float('inf')
            for neighbor in self._get_neighbors(s):
                cost = self._cost(s, neighbor)
                min_rhs = min(min_rhs, cost + self.g.get(neighbor, float('inf')))
            self.rhs[s] = min_rhs # Update the lookahead
            
        # If the cell is Inconsistent (g != rhs), it means the map changed and the math is broken here.
        # Add it to the queue so _compute_shortest_path can fix it.
        if self.g.get(s, float('inf')) != self.rhs.get(s, float('inf')):
            heapq.heappush(self.U, (self._key(s), s))

    def _compute_shortest_path(self):
        """
        The main D* Lite expansion loop.
        It runs until the Start cell is Consistent (g == rhs) and the queue is empty.
        """
        nodes_explored = 0
        # Loop condition: Queue is not empty AND (Start is inconsistent OR Start's key is greater than the lowest key in queue)
        while self.U and (self._key(self.U[0][1]) < self._key(self.start) or self.rhs.get(self.start, float('inf')) != self.g.get(self.start, float('inf'))):
            
            if nodes_explored > 5000:
                log_pathfinder("D* Lite taking too long, aborting!")
                return False
                
            k_old, s = heapq.heappop(self.U) # Pull the cell with the lowest key
            k_new = self._key(s)             # Recalculate its key right now
            
            if k_old < k_new:
                # The cell got worse (cost increased) since it was put in the queue. 
                # Push it back in with the new, higher key.
                heapq.heappush(self.U, (k_new, s))
                
            elif self.g.get(s, float('inf')) > self.rhs.get(s, float('inf')):
                # The cell is Underconsistent (g > rhs). Usually means a wall disappeared.
                # Fix it by making g equal to rhs, then update its neighbors.
                self.g[s] = self.rhs.get(s, float('inf'))
                nodes_explored += 1
                for neighbor in self._get_neighbors(s):
                    if 0 <= neighbor[0] < self.map_grid.shape[0] and 0 <= neighbor[1] < self.map_grid.shape[1]:
                        self._update_vertex(neighbor)
            else:
                # The cell is Overconsistent (g < rhs). Usually means a wall appeared.
                # Set g to infinity (make it impassable) and update its neighbors and itself.
                g_old = self.g.get(s, float('inf'))
                self.g[s] = float('inf')
                for neighbor in self._get_neighbors(s):
                    if 0 <= neighbor[0] < self.map_grid.shape[0] and 0 <= neighbor[1] < self.map_grid.shape[1]:
                        self._update_vertex(neighbor)
                self._update_vertex(s)
                
        return True

    def find_path(self, start_grid, goal_grid, map_grid):
        """
        Main entry point called by the Controller.
        Detects what changed (robot moved or map changed) and replans efficiently.
        """
        log_pathfinder(f"D* Lite: start={start_grid} goal={goal_grid} map_shape={map_grid.shape}")
        
        self.map_grid = map_grid
        
        # --- SCENARIO 1: Brand new goal or first boot ---
        # We must initialize the search from scratch.
        if self.goal != goal_grid or self.prev_map is None:
            log_pathfinder("D* Lite: Initializing fresh search...")
            self.U = []
            self.g = {}
            self.rhs = {}
            self.km = 0
            self.goal = goal_grid
            self.start = start_grid
            self.prev_map = map_grid.copy()
            
            # The cost to be at the goal is 0. Start the search here.
            self.rhs[goal_grid] = 0
            heapq.heappush(self.U, (self._key(goal_grid), goal_grid))
            
            if not self._compute_shortest_path():
                return []
                
        # --- SCENARIO 2: The robot moved ---
        # We don't replan! We just add the distance the robot moved to `km`.
        # This shifts the heuristic math without touching the queue. Massive time saver.
        elif self.start != start_grid:
            log_pathfinder("D* Lite: Robot moved, updating heuristic...")
            self.km += self._calculate_heuristic(self.start, start_grid)
            self.start = start_grid

        # --- SCENARIO 3: The map changed (A new wall appeared!) ---
        # This is where D* Lite shines. We use NumPy to instantly find exactly which pixels changed.
        if self.prev_map is not None and self.prev_map.shape == map_grid.shape:
            changed_cells = []
            # Fast numpy diff to find exactly which cells changed
            diff = self.prev_map != map_grid
            if diff.any():
                changed_indices = np.argwhere(diff)
                for idx in changed_indices:
                    s = tuple(idx)
                    changed_cells.append(s)
                    # Patch the math at the changed cell and its neighbors
                    self._update_vertex(s)
                    for neighbor in self._get_neighbors(s):
                        if 0 <= neighbor[0] < self.map_grid.shape[0] and 0 <= neighbor[1] < self.map_grid.shape[1]:
                            self._update_vertex(neighbor)
                            
                log_pathfinder(f"D* Lite: Map changed! Updated {len(changed_cells)} cells. Replanning...")
                
                # Run the expansion loop. It will only touch the broken cells, finishing instantly.
                if not self._compute_shortest_path():
                    return []
                    
        # If the map expanded (grid got larger), just update the reference. Coordinates are still valid.
        elif self.prev_map is not None and self.prev_map.shape != map_grid.shape:
            log_pathfinder("D* Lite: Map expanded. Adjusting...")
            self.prev_map = map_grid.copy()

        self.prev_map = map_grid.copy()

        # Extract the path
        path = self._reconstruct_path()
        log_pathfinder(f"D* Lite: Path found! Length: {len(path)}")
        return path

    def _reconstruct_path(self):
        """
        Walk from Start to Goal using the lowest g-scores.
        D* Lite doesn't have an origin_dict like A*. It only has g-scores (cost to goal).
        We just look at the neighbors and step onto the one with the lowest cost.
        (Greedy Walk)
        """
        if self.g.get(self.start, float('inf')) == float('inf'):
            log_pathfinder("D* Lite: No path exists!")
            return []
            
        path = []
        curr = self.start
        path.append(curr)
        
        max_steps = 2000 
        steps = 0
        
        while curr != self.goal and steps < max_steps:
            steps += 1
            
            min_cost = float('inf')
            best_next = None
            
            # Find the neighbor with the lowest (cost to move + g-score)
            for neighbor in self._get_neighbors(curr):
                if 0 <= neighbor[0] < self.map_grid.shape[0] and 0 <= neighbor[1] < self.map_grid.shape[1]:
                    cost = self._cost(curr, neighbor)
                    total_cost = cost + self.g.get(neighbor, float('inf'))
                    
                    if total_cost < min_cost:
                        min_cost = total_cost
                        best_next = neighbor
                        
            if best_next is None or min_cost == float('inf'):
                log_pathfinder("D* Lite: Path extraction failed (dead end)!")
                return []
                
            curr = best_next
            path.append(curr)
            
        return path

    def _calculate_heuristic(self, cell_a, cell_b):
        """Helper method to calculate the 'h' score using Octile Distance."""
        dx = abs(cell_a[0] - cell_b[0])
        dy = abs(cell_a[1] - cell_b[1])
        return max(dx, dy) + (1.414 - 1) * min(dx, dy)