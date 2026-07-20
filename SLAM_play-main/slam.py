import math
from debug import log_slam


class GridBasedSLAM:
    def __init__(self, initial_grid_width, initial_grid_height):
        # Vector-based storage: dictionary mapping (x,y) grid coordinates to log-odds values
        # Only stores occupied cells to save memory and improve performance
        self.occupied_cells = {}  # {(x, y): log_odds_value}
        self.grid_width = initial_grid_width
        self.grid_height = initial_grid_height
        log_slam(f"SLAM grid initialized: {initial_grid_width}x{initial_grid_height} (vector-based storage)")

    def expand_occupancy_grid(self, new_width, new_height):
        log_slam(f"Expanding grid: {self.grid_width}x{self.grid_height} -> {new_width}x{new_height}")
        # Vector-based storage doesn't need to copy data, just update dimensions
        self.grid_width, self.grid_height = new_width, new_height

    def world_to_grid(self, world_position):
        """Converts world coordinates to grid coordinates."""
        grid_x = int(world_position[0] // 10)
        grid_y = int(world_position[1] // 10)
        return grid_x, grid_y

    def _get_cell_value(self, x, y):
        """Get log-odds value for a cell, returns 0.0 if not stored (unknown)."""
        return self.occupied_cells.get((x, y), 0.0)

    def _set_cell_value(self, x, y, value):
        """Set log-odds value for a cell."""
        if value == 0.0:
            # Remove cell if value is 0 (unknown)
            self.occupied_cells.pop((x, y), None)
        else:
            self.occupied_cells[(x, y)] = value

    def sensor_update(self, robot_pose, sensor_bearing, sensor_data, max_sensor_range=200):
        grid_x, grid_y = self.world_to_grid(robot_pose)
        log_slam(f"sensor_update: robot=({robot_pose[0]:.1f}, {robot_pose[1]:.1f}) grid=({grid_x}, {grid_y}) bearing={sensor_bearing:.1f}° range={sensor_data:.1f}")

        if grid_x >= self.grid_width or grid_y >= self.grid_height:
            self.expand_occupancy_grid(max(grid_x + 10, self.grid_width), max(grid_y + 10, self.grid_height))

        # Log-Odds Increment Values
        L_FREE = -0.3     # Confidence added when a ray passes through a cell
        L_OCCUPIED = 0.5  # Confidence added when a ray hits an obstacle
        L_CLAMP = 5.0     # Maximum/Minimum confidence limit (prevents infinity)

        # Mark robot's current position as free
        current_value = self._get_cell_value(grid_x, grid_y)
        new_value = current_value + L_FREE
        if abs(new_value) >= L_CLAMP:
            new_value = L_CLAMP if new_value > 0 else -L_CLAMP
        self._set_cell_value(grid_x, grid_y, new_value)

        # Simulate clear space (Ray tracing)
        cells_marked_free = 0
        for ray_distance in range(10, int(sensor_data), 10):
            clear_x = grid_x + int(ray_distance * math.cos(math.radians(sensor_bearing)) / 10)
            clear_y = grid_y + int(ray_distance * math.sin(math.radians(sensor_bearing)) / 10)

            if clear_x >= self.grid_width or clear_y >= self.grid_height:
                self.expand_occupancy_grid(max(clear_x + 10, self.grid_width), max(clear_y + 10, self.grid_height))

            # Subtract confidence (more likely to be free)
            current_value = self._get_cell_value(clear_x, clear_y)
            new_value = current_value + L_FREE
            if abs(new_value) >= L_CLAMP:
                new_value = L_CLAMP if new_value > 0 else -L_CLAMP
            self._set_cell_value(clear_x, clear_y, new_value)
            cells_marked_free += 1

        # If sensor detects an obstacle, mark it
        if sensor_data < max_sensor_range:
            obstacle_x = grid_x + int(round(sensor_data * math.cos(math.radians(sensor_bearing)) / 10.0))
            obstacle_y = grid_y + int(round(sensor_data * math.sin(math.radians(sensor_bearing)) / 10.0))

            # NEW: Prevent marking the robot's own cell as an obstacle
            if obstacle_x == grid_x and obstacle_y == grid_y:
                pass # Skip! The obstacle is too close / rounding put it on top of us.
            elif obstacle_x >= self.grid_width or obstacle_y >= self.grid_height:
                self.expand_occupancy_grid(max(obstacle_x + 10, self.grid_width), max(obstacle_y + 10, self.grid_height))
                current_value = self._get_cell_value(obstacle_x, obstacle_y)
                new_value = current_value + L_OCCUPIED
                if abs(new_value) >= L_CLAMP:
                    new_value = L_CLAMP if new_value > 0 else -L_CLAMP
                self._set_cell_value(obstacle_x, obstacle_y, new_value)
            else:
                current_value = self._get_cell_value(obstacle_x, obstacle_y)
                new_value = current_value + L_OCCUPIED
                if abs(new_value) >= L_CLAMP:
                    new_value = L_CLAMP if new_value > 0 else -L_CLAMP
                self._set_cell_value(obstacle_x, obstacle_y, new_value)

            log_slam(f"  Marked obstacle at grid=({obstacle_x}, {obstacle_y}), {cells_marked_free} cells marked free")
        else:
            log_slam(f"  No obstacle detected (max range), {cells_marked_free} cells marked free")

    def lidar_update(self, robot_pose, lidar_data):
        """Loops through a 360-degree LiDAR scan and updates the map."""
        for angle, distance in lidar_data:
            # Call your existing single-ray logic for every ray in the scan!
            self.sensor_update(robot_pose, angle, distance)

    def get_map(self):
        # Return a numpy array for compatibility with existing code
        import numpy as np
        map_array = np.zeros((self.grid_width, self.grid_height), dtype=float)
        for (x, y), value in self.occupied_cells.items():
            if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                map_array[x, y] = value
        return map_array

    @property
    def map_size(self):
        """Return the map dimensions as a tuple (width, height)."""
        return (self.grid_width, self.grid_height)
