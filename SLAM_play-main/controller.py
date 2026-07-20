import math
import random as rand
import pygame
import heapq
from collections import deque
from pathfinder import Pathfinder
from debug import log_controller


class Controller:
    def __init__(self, robot=None, slam=None):
        self.robot = robot
        self.slam = slam
        self.current_goal = None
        self.current_goal_patch = []
        self.state = "idle"
        self.last_position = None
        self.stuck_since = None
        self.goal_start_time = 0
        self.stuck_counter = 0  # Counter for tracking if robot is stuck
        self.goal_attempts = {}       # Tracks how many times a goal has failed
        self.blacklisted_goals = set() # Stores goals that failed 3 times

        # Pathfinding
        self.pathfinder = Pathfinder()
        self.current_path = []  # Stores the A* path as grid coordinates

        # Avoidance variables
        self.avoiding = False
        self.avoidance_target_angle = 0
        self.avoidance_timer = 0
        log_controller("Controller initialized")

    def _grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates."""
        return (gx * 10 + 5, gy * 10 + 5)

    def _get_path_target(self, frontier_grid):
        """Find a known free cell adjacent to the frontier for A* to target."""
        map_grid = self.slam.get_map()
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = frontier_grid[0] + dx, frontier_grid[1] + dy
            if 0 <= nx < map_grid.shape[0] and 0 <= ny < map_grid.shape[1]:
                # If it's highly confident free space, we can pathfind to it
                if map_grid[nx, ny] < -1.0:
                    log_controller(f"_get_path_target: frontier={frontier_grid} -> target={(nx, ny)}")
                    return (nx, ny)
        log_controller(f"_get_path_target: frontier={frontier_grid} -> NO free neighbor found")
        return None # Fallback if no free neighbor found

    def _find_raw_frontiers(self):
        """BFS to find all individual frontier cells in grid coordinates.
        A frontier is defined as an unknown cell (0) directly adjacent to a free cell (1).
        """
        # Convert the robot's float world position to integer grid coordinates to start the search
        start_gx = int(self.robot.position[0]) // 10
        start_gy = int(self.robot.position[1]) // 10

        queue = deque([(start_gx, start_gy)])
        visited = set([(start_gx, start_gy)]) # Track visited cells to prevent infinite loops
        frontiers = set() # Use a set for fast lookups later

        map_grid = self.slam.get_map()
        max_x = map_grid.shape[0] - 1
        max_y = map_grid.shape[1] - 1

        # 8-way directions for navigating free space (allows diagonal movement/searching)
        directions_8 = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]

        while queue:
            curr_gx, curr_gy = queue.popleft()

            # If it's the starting cell, always search it. 
            # Otherwise, only expand from Known Free Space (< -0.8)
            if curr_gx != start_gx or curr_gy != start_gy:
                if map_grid[curr_gx, curr_gy] >= -0.8:
                    continue

            for dx, dy in directions_8:
                nx, ny = curr_gx + dx, curr_gy + dy

                if 0 <= nx <= max_x and 0 <= ny <= max_y and (nx, ny) not in visited:
                    visited.add((nx, ny))

                    # UNKNOWN SPACE is near 0 (e.g., > -0.5)
                    if -0.8 <= map_grid[nx, ny] <= 0.8:
                        frontiers.add((nx, ny))
                    # FREE SPACE
                    elif map_grid[nx, ny] <= -0.8:
                        queue.append((nx, ny))

        log_controller(f"_find_raw_frontiers: found {len(frontiers)} raw frontier cells")
        return frontiers

    def _cluster_cells(self, cells_set):
        """Group adjacent grid cells into connected clusters.
        Uses 4-way connectivity (Up/Down/Left/Right) so we don't accidentally
        connect two frontiers diagonally across a thin wall corner.
        """
        if not cells_set:
            return []

        clusters = []
        unvisited = set(cells_set) # Copy the set so we can remove cells as we group them
        directions_8 = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]

        # Keep going until every cell has been assigned to a cluster
        while unvisited:
            start_f = unvisited.pop() # Grab any unvisited cell to start a new cluster
            cluster = []
            q = deque([start_f])

            # Mini-BFS to find all cells connected to this starting cell
            while q:
                curr = q.popleft()
                cluster.append(curr)

                cx, cy = curr
                for dx, dy in directions_8:
                    neighbor = (cx + dx, cy + dy)
                    if neighbor in unvisited:
                        unvisited.remove(neighbor) # Remove so it's not checked again
                        q.append(neighbor)

            clusters.append(cluster)

        log_controller(f"_cluster_cells: formed {len(clusters)} clusters (sizes: {[len(c) for c in clusters]})")
        return clusters

    def _get_largest_valid_clusters(self, clusters, min_size=3):
        """Filter out small clusters and return the largest ones."""
        valid_clusters = [c for c in clusters if len(c) >= min_size]

        if not valid_clusters:
            log_controller(f"_get_largest_valid_clusters: no valid clusters (min_size={min_size})")
            return []

        largest_5 = heapq.nlargest(5, valid_clusters, key=len)
        # Safe logging in case there are less than 5 clusters
        sizes = [len(c) for c in largest_5]
        log_controller(f"_get_largest_valid_clusters: largest cluster sizes: {sizes}")
        return largest_5

    def frontier_finder(self):
        """The main pipeline to find the top largest connected patches of frontiers."""
        # 1. Get raw frontier cells (grid coords)
        raw_frontiers = self._find_raw_frontiers()
        if not raw_frontiers:
            return []

        # 2. Group them into connected clusters
        clusters = self._cluster_cells(raw_frontiers)

        # 3. Filter out small clusters and get the top 5 largest ones
        largest_clusters = self._get_largest_valid_clusters(clusters, min_size=3)
        
        return largest_clusters

    def set_goal(self, robot_position, pick_random=False):
        """Set a new goal based on Utility (Reward / Cost)."""
        candidate_clusters = self.frontier_finder()

        if not candidate_clusters:
            log_controller("set_goal: no frontier patch found, going idle")
            self.state = "idle"
            self.current_goal = None
            self.current_goal_patch = []
            self.current_path = []
            return

        start_grid = (int(self.robot.position[0]) // 10, int(self.robot.position[1]) // 10)
        
        best_score = -1
        best_cluster = None
        best_path = []

        for cluster in candidate_clusters:
            # 1. Calculate Reward (Size of the unknown area)
            reward = len(cluster)
            
            # 2. Calculate Cost (A* path length + Turn penalty)
            # --- NEW: CALCULATE CENTROID ---
            avg_x = sum(cell[0] for cell in cluster) / len(cluster)
            avg_y = sum(cell[1] for cell in cluster) / len(cluster)
            centroid = (int(round(avg_x)), int(round(avg_y)))
            
            # Tell A* to path to the free cell adjacent to the CENTROID
            target_free_cell = self._get_path_target(centroid)
            
            cost = 9999
            path = []
            if target_free_cell:
                path = self.pathfinder.find_path(start_grid, target_free_cell, self.slam.get_map())
                if path: 
                    # --- DISTANCE COST ---
                    distance_cost = len(path)
                    
                    # --- TOTAL TURN COST ---
                    total_turn_cost = 0.0
                    
                    # 1. Initial turn from robot's current angle to the first path step
                    first_step_world = self._grid_to_world(path[0][0], path[0][1])
                    angle_to_first_step = math.degrees(
                        math.atan2(first_step_world[1] - self.robot.position[1],
                                   first_step_world[0] - self.robot.position[0]))
                    
                    initial_angle_diff = abs((angle_to_first_step - self.robot.angle + 180) % 360 - 180)
                    total_turn_cost += initial_angle_diff / 20.0 
                    
                    # 2. Internal turns along the path
                    for i in range(1, len(path) - 1):
                        p_prev = path[i-1]
                        p_curr = path[i]
                        p_next = path[i+1]
                        
                        # Direction vectors
                        dir1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
                        dir2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
                        
                        # If the direction changed, it's a turn!
                        if dir1 != dir2:
                            # Calculate the exact angle of the turn (e.g., 45, 90, or 180 degrees)
                            angle1 = math.degrees(math.atan2(dir1[1], dir1[0]))
                            angle2 = math.degrees(math.atan2(dir2[1], dir2[0]))
                            turn_angle = abs((angle2 - angle1 + 180) % 360 - 180)
                            
                            # Add penalty for this turn
                            total_turn_cost += turn_angle / 20.0
                    
                    # --- TOTAL COST ---
                    cost = distance_cost + total_turn_cost
            
            # 3. Calculate Score
            if cost > 0 and cost < 9999:
                score = reward / cost
            else:
                score = 0 # Unreachable, ignore
                
            # 4. Keep track of the best scoring cluster
            if score > best_score:
                best_score = score
                best_cluster = cluster
                best_path = path

        # If we found a valid cluster, set it as the goal
        if best_cluster:
            # --- NEW: SET GOAL TO CENTROID ---
            avg_x = sum(cell[0] for cell in best_cluster) / len(best_cluster)
            avg_y = sum(cell[1] for cell in best_cluster) / len(best_cluster)
            centroid_grid_goal = (int(round(avg_x)), int(round(avg_y)))
            
            # --- NEW: FALLBACK LOGIC ---
            # If best_cluster was chosen by the fallback (no A* path found because it's too close)
            if not best_path:
                log_controller("Centroid too close! Using furthest point to escape.")
                # Pick the furthest cell in the cluster from the robot
                centroid_grid_goal = max(best_cluster, key=lambda f: math.hypot(f[0] - start_grid[0], f[1] - start_grid[1]))
            
            self.current_goal = self._grid_to_world(centroid_grid_goal[0], centroid_grid_goal[1])
            self.current_goal_patch = [self._grid_to_world(gx, gy) for gx, gy in best_cluster]
            
            # Use the path we already calculated in the loop!
            self.current_path = best_path
            
            log_controller(f"set_goal: new goal={self.current_goal}, patch size={len(best_cluster)}, score={best_score:.2f}, path steps={len(self.current_path)}")
            
            self.state = "moving_to_goal"
            self.goal_start_time = pygame.time.get_ticks()
        else:
            log_controller("set_goal: couldn't find a valid best cluster, going idle")
            self.state = "idle"
            self.current_goal = None
            self.current_goal_patch = []
            self.current_path = []

    def update(self, sensor_data, robot_position):
        """Update the controller's state based on sensor data and robot's position."""
        if self.state == "idle":
            self.set_goal(robot_position)

        elif self.state == "moving_to_goal":

            if pygame.time.get_ticks() - self.goal_start_time > 8000:
                log_controller("Goal timeout! Picking a new point in the frontier.")
                
                # --- NEW: TRACK FAILED ATTEMPTS ---
                if self.current_goal:
                    goal_grid = (int(self.current_goal[0] // 10), int(self.current_goal[1] // 10))
                    self.goal_attempts[goal_grid] = self.goal_attempts.get(goal_grid, 0) + 1
                    
                    if self.goal_attempts[goal_grid] >= 3:
                        self.blacklisted_goals.add(goal_grid)
                        log_controller(f"Goal {goal_grid} failed 3 times. Blacklisting!")

                self.set_goal(robot_position, pick_random=True)
                return

            # Move (only reached if not avoiding)
            self._move_towards_goal(sensor_data)

            if self.current_goal:
                goal_grid = (int(self.current_goal[0]) // 10, int(self.current_goal[1]) // 10)
                map_grid = self.slam.get_map()
                
                # Ensure goal is within bounds just in case
                if 0 <= goal_grid[0] < map_grid.shape[0] and 0 <= goal_grid[1] < map_grid.shape[1]:                    
                    # Calculate actual distance from robot to the goal coordinate
                    dist_to_goal = math.hypot(self.current_goal[0] - self.robot.position[0], 
                                            self.current_goal[1] - self.robot.position[1])
                    
                    # If the robot is within 15 pixels (1.5 grid cells) of the goal
                    if dist_to_goal < 30:
                        log_controller(f"GOAL REACHED at {self.current_goal}! (dist={dist_to_goal:.1f})")
                        self.state = "idle"
                        self.current_goal = None
                        self.current_goal_patch = []
                        self.current_path = []
                else:
                    # Out of bounds, just clear it
                    self.state = "idle"
                    self.current_goal = None
                    self.current_goal_patch = []
                    self.current_path = []

    def _move_towards_goal(self, sensor_data):
        """Move the robot along the A* path using look-ahead for smooth driving."""

        # If there is no A* path, just point at the final goal as a fallback
        if not self.current_path:
            if self.current_goal:
                self.robot.target_angle = math.degrees(
                    math.atan2(self.current_goal[1] - self.robot.position[1],
                            self.current_goal[0] - self.robot.position[0]))
                self.robot.rotate_towards(self.robot.target_angle)

                angle_diff = (self.robot.target_angle - self.robot.angle + 180) % 360 - 180
                if abs(angle_diff) < 45 and sensor_data > 50:
                    self.robot.move_forward()
            return

        # Get robot's current grid position for the LoS check
        start_gx = int(self.robot.position[0]) // 10
        start_gy = int(self.robot.position[1]) // 10
        start_grid_pos = (start_gx, start_gy)
        map_grid = self.slam.get_map()

        # 1. LOOK-AHEAD LOGIC: Find the furthest point in the path that is roughly straight ahead AND clear of walls
        target_index = 0
        first_step_world = self._grid_to_world(self.current_path[0][0], self.current_path[0][1])
        target_angle = math.degrees(
            math.atan2(first_step_world[1] - self.robot.position[1],
                       first_step_world[0] - self.robot.position[0]))

        # Check the rest of the path to see if we can skip points
        for i in range(1, len(self.current_path)):
            future_world = self._grid_to_world(self.current_path[i][0], self.current_path[i][1])
            future_angle = math.degrees(
                math.atan2(future_world[1] - self.robot.position[1],
                           future_world[0] - self.robot.position[0]))
            
            # If this future point is in roughly the same direction (within 15 degrees)
            angle_diff = abs((future_angle - target_angle + 180) % 360 - 180)
            if angle_diff < 15:
                # CRITICAL: Check if we would hit a wall by cutting the corner
                if self._is_line_clear(start_grid_pos, self.current_path[i], map_grid):
                    target_index = i  # Line is clear, we can skip straight to this point!
                else:
                    break # Obstacle in the way! Stop looking further, stick to the path
            else:
                break # The path is bending, stop looking further

        # 2. Aim at the chosen look-ahead point
        look_ahead_world = self._grid_to_world(self.current_path[target_index][0], self.current_path[target_index][1])
        self.robot.target_angle = math.degrees(
            math.atan2(look_ahead_world[1] - self.robot.position[1],
                    look_ahead_world[0] - self.robot.position[0]))
        self.robot.rotate_towards(self.robot.target_angle)

        # 3. Clean up the path: Pop points we've driven past
        while len(self.current_path) > 1:
            next_step_world = self._grid_to_world(self.current_path[0][0], self.current_path[0][1])
            dist = math.hypot(next_step_world[0] - self.robot.position[0], next_step_world[1] - self.robot.position[1])
            if dist < 10: # If we are close to the first point in the list, remove it
                self.current_path.pop(0)
            else:
                break 

        # 4. Move forward ONLY if the sensor says it's safe and we are facing the right way
        angle_diff = (self.robot.target_angle - self.robot.angle + 180) % 360 - 180
        if abs(angle_diff) < 30 and sensor_data > 50:
            self.robot.move_forward()
            self.stuck_counter = 0 # Reset counter, we moved!
        elif sensor_data < 50 and self.current_path:
            # Sensor is hitting the brakes! We might be stuck against a wall A* didn't account for.
            self.stuck_counter += 1
            if self.stuck_counter > 20: # If stuck for 20 frames, force a recalculation
                log_controller("Stuck against wall! Clearing path to recalculate.")
                self.current_path = []
                self.stuck_counter = 0
    
    def _is_line_clear(self, start_grid, end_grid, map_grid):
        """Check if the straight line between two grid cells is free of obstacles.
        Uses Bresenham's algorithm to ensure NO grid cells are skipped."""
        x0, y0 = start_grid
        x1, y1 = end_grid
        
        # Calculate absolute differences
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        # Determine step direction
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        # Initialize error term
        err = dx - dy
        x, y = x0, y0
        
        while True:
            # 1. Check bounds
            if not (0 <= x < map_grid.shape[0] and 0 <= y < map_grid.shape[1]):
                return False # Out of bounds
                
            # 2. Check if current cell is an obstacle or unknown
            # (Using your log-odds threshold: >= -0.8 means not confidently free)
            if map_grid[x, y] >= -0.8:
                return False
                
            # 3. If we reached the end cell, we are done and clear!
            if x == x1 and y == y1:
                return True
                
            # 4. Step to the next cell using Bresenham's math
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy