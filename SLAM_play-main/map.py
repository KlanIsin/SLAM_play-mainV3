import pygame
import random

class World:
    def __init__(self):
        self.width = 1000  # Define the world width
        self.height = 1000  # Define the world height

        # World boundaries (obstacles)
        self.obstacles = [
            pygame.Rect(0, 0, self.width, 10),  # Top boundary
            pygame.Rect(0, 0, 10, self.height),  # Left boundary
            pygame.Rect(0, self.height - 10, self.width, 10),  # Bottom boundary
            pygame.Rect(self.width - 10, 0, 10, self.height)  # Right boundary
        ]

        # Generate 3-5 random obstacles
        self.generate_random_obstacles()

    def generate_random_obstacles(self):
        num_obstacles = random.randint(3, 5)  # Random number of obstacles between 3 and 5
        for _ in range(num_obstacles):
            # Generate random position and size for each obstacle
            obstacle_width = random.randint(30, 70)  # Random width between 30 and 70
            obstacle_height = random.randint(30, 70)  # Random height between 30 and 70

            # Ensure obstacles are placed within the world bounds
            obstacle_x = random.randint(20, self.width - obstacle_width - 20)
            obstacle_y = random.randint(20, self.height - obstacle_height - 20)

            # Create and add the obstacle to the list
            self.obstacles.append(pygame.Rect(obstacle_x, obstacle_y, obstacle_width, obstacle_height))
