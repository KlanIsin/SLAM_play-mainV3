import pygame
import random
import math
from debug import log_map

class World:
    def __init__(self):
        self.width = 1000
        self.height = 1000
        
        # World boundaries (outer walls)
        self.obstacles = [
            pygame.Rect(0, 0, self.width, 10),  # Top
            pygame.Rect(0, 0, 10, self.height),  # Left
            pygame.Rect(0, self.height - 10, self.width, 10),  # Bottom
            pygame.Rect(self.width - 10, 0, 10, self.height)  # Right
        ]

        self.generate_floorplan()

    def generate_floorplan(self):
        log_map("Generating floorplan with rooms and hallways...")
        
        # 1. Create Horizontal Hallways (with door gaps)
        # Divide the map into 3 horizontal sections
        for i in range(2):
            y = (i + 1) * (self.height // 3)
            wall_y = y - 5
            
            # Left wall segment
            gap_start = random.randint(100, 400)
            self.obstacles.append(pygame.Rect(10, wall_y, gap_start, 10))
            
            # Right wall segment (leaving a doorway in the middle)
            gap_end = gap_start + 100 # 100px wide doorway
            self.obstacles.append(pygame.Rect(gap_end, wall_y, self.width - gap_end - 10, 10))

        # 2. Create Vertical Walls (with door gaps)
        # Divide the map into 3 vertical sections
        for i in range(2):
            x = (i + 1) * (self.width // 3)
            wall_x = x - 5
            
            gap_start = random.randint(100, 400)
            self.obstacles.append(pygame.Rect(wall_x, 10, 10, gap_start))
            
            gap_end = gap_start + 100
            self.obstacles.append(pygame.Rect(wall_x, gap_end, 10, self.height - gap_end - 10))

        # 3. Add Clutter (Furniture/Boxes)
        num_clutter = random.randint(20, 35)
        for _ in range(num_clutter):
            clutter_w = random.randint(10, 30)
            clutter_h = random.randint(10, 30)
            
            # Random position, but keep it away from the exact center (robot spawn)
            cx = random.randint(50, self.width - 50)
            cy = random.randint(50, self.height - 50)
            
            if math.hypot(cx - self.width//2, cy - self.height//2) < 100:
                continue # Too close to spawn, skip
                
            self.obstacles.append(pygame.Rect(cx, cy, clutter_w, clutter_h))
            
        # 4. Add a few long maze-like walls
        for _ in range(5):
            is_horizontal = random.choice([True, False])
            if is_horizontal:
                wall_w = random.randint(100, 300)
                wall_h = 10
                wx = random.randint(50, self.width - wall_w - 50)
                wy = random.randint(50, self.height - 50)
            else:
                wall_w = 10
                wall_h = random.randint(100, 300)
                wx = random.randint(50, self.width - 50)
                wy = random.randint(50, self.height - wall_h - 50)
                
            self.obstacles.append(pygame.Rect(wx, wy, wall_w, wall_h))

        log_map(f"Total obstacles (including boundaries): {len(self.obstacles)}")