import math
import random as rand
import pygame
from collections import deque

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
        
        # Avoidance variables
        self.avoiding = False
        self.avoidance_target_angle = 0
        self.avoidance_timer = 0

    def _grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates."""
        return (gx * 10 + 5, gy * 10 + 5)

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

            # We only expand our search from cells we know are free (1)
            if map_grid[curr_gx, curr_gy] != 1:
                continue

            for dx, dy in directions_8:
                nx, ny = curr_gx + dx, curr_gy + dy
                
                # If the neighbor is within map bounds and we haven't checked it yet
                if 0 <= nx <= max_x and 0 <= ny <= max_y and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    
                    # If the neighbor is unknown (0), we found a frontier!
                    if map_grid[nx, ny] == 0: 
                        frontiers.add((nx, ny))
                    # If the neighbor is free (1), add it to the queue to keep expanding the search
                    elif map_grid[nx, ny] == 1:
                        queue.append((nx, ny))
                        
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
        directions_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

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
                for dx, dy in directions_4:
                    neighbor = (cx + dx, cy + dy)
                    if neighbor in unvisited:
                        unvisited.remove(neighbor) # Remove so it's not checked again
                        q.append(neighbor)
                        
            clusters.append(cluster)
            
        return clusters

    def _get_largest_valid_cluster(self, clusters, min_size=5):
        """Filter out small clusters and return the largest one.
        min_size ensures the robot ignores tiny 1 or 2 cell patches 
        (which are usually sensor noise or tiny corners it can't fit into).
        """
        valid_clusters = [c for c in clusters if len(c) >= min_size]
        
        if not valid_clusters:
            return None
            
        # Return the cluster with the highest length (most cells)
        return max(valid_clusters, key=len)

    def frontier_finder(self):
        """The main pipeline to find the largest connected patch of frontiers in world coordinates."""
        # 1. Get raw frontier cells (grid coords)
        raw_frontiers = self._find_raw_frontiers()
        if not raw_frontiers:
            return []

        # 2. Group them into connected clusters
        clusters = self._cluster_cells(raw_frontiers)
        
        # 3. Filter out small clusters and get the largest one
        largest_cluster = self._get_largest_valid_cluster(clusters, min_size=5)
        if not largest_cluster:
            return []
            
        # 4. Convert the winning cluster back to world coordinates for navigation
        return [self._grid_to_world(gx, gy) for gx, gy in largest_cluster]

    def set_goal(self, robot_position, pick_random=False):
        """Set a new goal. If pick_random is True, pick a different point in the patch."""
        largest_frontier_patch = self.frontier_finder()
        
        if largest_frontier_patch:
            self.current_goal_patch = largest_frontier_patch  # Save patch for visualization
            
            if pick_random and len(largest_frontier_patch) > 1:
                # Filter out the current goal so we pick a NEW point some ways away
                choices = [f for f in largest_frontier_patch if f != self.current_goal]
                if choices:
                    self.current_goal = rand.choice(choices)
                else:
                    self.current_goal = rand.choice(largest_frontier_patch)
            else:
                closest_frontier = min(largest_frontier_patch, key=lambda f: math.hypot(f[0] - robot_position[0], f[1] - robot_position[1]))
                self.current_goal = closest_frontier
                
            self.state = "moving_to_goal"
            self.goal_start_time = pygame.time.get_ticks()  # Record when we set this goal
        else:
            self.state = "idle"
            self.current_goal = None
            self.current_goal_patch = []  # Clear patch
    
    def update(self, sensor_data, robot_position):
        """Update the controller's state based on sensor data and robot's position."""
        if self.state == "idle":
            self.set_goal(robot_position)

        elif self.state == "moving_to_goal":

            if pygame.time.get_ticks() - self.goal_start_time > 15000:
                print("Goal timeout! Picking a new point in the frontier.")
                self.set_goal(robot_position, pick_random=True)
                return
            
            # PRIORITY 1: Avoidance. If we are avoiding, STOP here.
            if self.obstacle_avoidance(sensor_data):
                return
            
            # PRIORITY 2: Move (only reached if not avoiding)
            self._move_towards_goal()
            
            # Check if reached goal
            if self.current_goal and math.hypot(self.current_goal[0] - robot_position[0], self.current_goal[1] - robot_position[1]) < 30:
                self.state = "idle"
                self.current_goal = None
                self.current_goal_patch = []

    def obstacle_avoidance(self, sensor_data):
        """Returns True if avoiding, False if safe to move towards goal."""
        if self.avoiding:
            self.avoidance_timer += 1
            self.robot.rotate_towards(self.avoidance_target_angle)
            
            if self.avoidance_timer > 30:
                self.avoiding = False
                self.avoidance_timer = 0
            return True # Still avoiding, don't move to goal
            
        else:
            if sensor_data < 50: 
                self.avoiding = True
                self.avoidance_target_angle = (self.robot.angle + 120) % 360
                self.robot.rotate_towards(self.avoidance_target_angle)
                self.avoidance_timer = 0
                return True # Started avoiding
            elif sensor_data < 100: 
                self.robot.rotate_towards(self.robot.angle + 10) 
                return True # Buffer zone, don't move to goal
                
        return False # Path is clear, safe to move to goal
    
    def _move_towards_goal(self):
        """Move the robot toward the current goal."""
        if self.current_goal:
            self.robot.target_angle = math.degrees(
                math.atan2(self.current_goal[1] - self.robot.position[1],
                        self.current_goal[0] - self.robot.position[0]))
            self.robot.rotate_towards(self.robot.target_angle)
            if math.hypot(self.current_goal[0] - self.robot.position[0],
                        self.current_goal[1] - self.robot.position[1]) > 5:
                self.robot.move_forward()