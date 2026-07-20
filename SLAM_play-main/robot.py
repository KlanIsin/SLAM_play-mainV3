import pygame
import math
import random
from debug import log_robot, log_sensor


class Robot:
    def __init__(self, position, angle):
        self.position = position
        self.angle = angle
        self.speed = 8  # Movement speed (adjust as needed)
        self.rotation_speed = 5  # Rotation speed (adjust as needed)
        self.radius = 10  # Robot size (visual purposes)
        log_robot(f"Robot initialized at pos=({position[0]}, {position[1]}) angle={angle}°")

    # Move forward
    def move_forward(self):
        old_x, old_y = self.position[0], self.position[1]
        self.position[0] += self.speed * math.cos(math.radians(self.angle))
        self.position[1] += self.speed * math.sin(math.radians(self.angle))
        log_robot(f"move_forward: ({old_x:.1f}, {old_y:.1f}) -> ({self.position[0]:.1f}, {self.position[1]:.1f})")

    # Move backward
    def move_backward(self):
        old_x, old_y = self.position[0], self.position[1]
        self.position[0] -= self.speed * math.cos(math.radians(self.angle))
        self.position[1] -= self.speed * math.sin(math.radians(self.angle))
        log_robot(f"move_backward: ({old_x:.1f}, {old_y:.1f}) -> ({self.position[0]:.1f}, {self.position[1]:.1f})")

    # Rotate left
    def rotate_left(self):
        self.angle -= self.rotation_speed
        if self.angle < 0:
            self.angle += 360
        log_robot(f"rotate_left: angle now {self.angle:.2f}°")

    # Rotate right
    def rotate_right(self):
        self.angle += self.rotation_speed
        if self.angle >= 360:
            self.angle -= 360
        log_robot(f"rotate_right: angle now {self.angle:.2f}°")

    def rotate_towards(self, target_angle):
        """Rotate the robot towards a specific angle."""
        self.target_angle = target_angle
        # Calculate the difference between the current angle and the target angle
        angle_difference = (self.target_angle - self.angle) % 360

        # Determine the shortest rotation direction
        if angle_difference > 180:
            log_robot(f"rotate_towards: target={target_angle:.2f}°, diff={angle_difference:.2f}°, turning LEFT")
            self.rotate_left()
        else:
            log_robot(f"rotate_towards: target={target_angle:.2f}°, diff={angle_difference:.2f}°, turning RIGHT")
            self.rotate_right()

    def simulate_ultrasonic(self, obstacles):
        """Simulates an ultrasonic sensor by detecting obstacles in the robot's direction."""
        max_distance = 200  # Maximum range of the ultrasonic sensor
        step_size = 5  # Step size for sensor simulation
        
        perfect_distance = max_distance # Assume clear initially

        # Scan forward in steps to check for obstacles
        for distance in range(0, max_distance, step_size):
            x = self.position[0] + distance * math.cos(math.radians(self.angle))
            y = self.position[1] + distance * math.sin(math.radians(self.angle))

            # Check for collision with any obstacle
            for obstacle in obstacles:
                if obstacle.collidepoint(x, y):
                    perfect_distance = distance  # Found the true distance
                    break # Stop checking other obstacles for this ray step
            if perfect_distance < max_distance:
                break # Stop casting the ray further

        # ==========================================
        # NOISE FIX: Prevent phantom walls at max range
        # ==========================================
        
        # If the perfect distance is exactly max, it saw NOTHING.
        # Return exactly max so SLAM knows it didn't hit a wall. No noise allowed here!
        if perfect_distance == max_distance:
            return max_distance
            
        # If it DID hit a wall, apply noise to that actual measurement
        noisy_distance = perfect_distance + random.gauss(0, 2.0)
        
        # Clamp the value so noise doesn't push it past max_distance (which would confuse SLAM)
        # or below 0.
        noisy_distance = max(5, min(noisy_distance, max_distance - 1))
        
        return noisy_distance

    def simulate_lidar(self, obstacles):
        """Simulates a LiDAR sensor by detecting obstacles in a 360-degree scan."""
        max_distance = 200  # Maximum range of the LiDAR sensor
        step_size = 2  # Smaller step size for higher resolution (typical for LiDAR)
        
        # Scan in 360 degrees with smaller step size for higher resolution
        # We'll sample at regular angular intervals
        num_samples = int(360 / step_size)  # Number of angular samples
        
        # Store the closest obstacle distance for each angle
        closest_distances = []
        
        for angle_step in range(num_samples):
            current_angle = (self.angle + angle_step * step_size) % 360
            
            perfect_distance = max_distance  # Assume clear initially
            
            # Scan forward in steps to check for obstacles at this angle
            for distance in range(0, max_distance, step_size):
                x = self.position[0] + distance * math.cos(math.radians(current_angle))
                y = self.position[1] + distance * math.sin(math.radians(current_angle))

                # Check for collision with any obstacle
                for obstacle in obstacles:
                    if obstacle.collidepoint(x, y):
                        perfect_distance = distance  # Found the true distance
                        break  # Stop checking other obstacles for this ray step
                if perfect_distance < max_distance:
                    break  # Stop casting the ray further

            # ==========================================
            # NOISE FIX: Prevent phantom walls at max range
            # ==========================================
            
            # If the perfect distance is exactly max, it saw NOTHING.
            # Return exactly max so SLAM knows it didn't hit a wall. No noise allowed here!
            if perfect_distance == max_distance:
                closest_distances.append(max_distance)
            else:
                # If it DID hit a wall, apply noise to that actual measurement
                noisy_distance = perfect_distance + random.gauss(0, 2.0)
                
                # Clamp the value so noise doesn't push it past max_distance (which would confuse SLAM)
                # or below 0.
                noisy_distance = max(5, min(noisy_distance, max_distance - 1))
                closest_distances.append(noisy_distance)
        
        # Return a list of (angle, distance) tuples for SLAM
        scan_results = []
        for i, dist in enumerate(closest_distances):
            angle = (self.angle + i * step_size) % 360
            scan_results.append((angle, dist))
        return scan_results