import pygame
import math

class Robot:
    def __init__(self, position, angle):
        self.position = position
        self.angle = angle
        self.speed = 2  # Movement speed (adjust as needed)
        self.rotation_speed = 2  # Rotation speed (adjust as needed)
        self.radius = 10  # Robot size (visual purposes)

    # Move forward
    def move_forward(self):
        self.position[0] += self.speed * math.cos(math.radians(self.angle))
        self.position[1] += self.speed * math.sin(math.radians(self.angle))

    # Move backward
    def move_backward(self):
        self.position[0] -= self.speed * math.cos(math.radians(self.angle))
        self.position[1] -= self.speed * math.sin(math.radians(self.angle))

    # Rotate left
    def rotate_left(self):
        self.angle -= self.rotation_speed
        if self.angle < 0:
            self.angle += 360

    # Rotate right
    def rotate_right(self):
        self.angle += self.rotation_speed
        if self.angle >= 360:
            self.angle -= 360

    def rotate_towards(self, target_angle):
        """Rotate the robot towards a specific angle."""
        self.target_angle = target_angle
        # Calculate the difference between the current angle and the target angle
        angle_difference = (self.target_angle - self.angle) % 360

        # Determine the shortest rotation direction
        if angle_difference > 180:
            self.rotate_left()
        else:
            self.rotate_right()

    def simulate_ultrasonic(self, obstacles):
        """Simulates an ultrasonic sensor by detecting obstacles in the robot's direction."""
        max_distance = 200  # Maximum range of the ultrasonic sensor
        step_size = 5  # Step size for sensor simulation

        # Scan forward in steps to check for obstacles
        for distance in range(0, max_distance, step_size):
            x = self.position[0] + distance * math.cos(math.radians(self.angle))
            y = self.position[1] + distance * math.sin(math.radians(self.angle))

            # Check for collision with any obstacle
            for obstacle in obstacles:
                if obstacle.collidepoint(x, y):
                    return distance  # Return distance to the obstacle

        return max_distance  # No obstacle detected within max range
