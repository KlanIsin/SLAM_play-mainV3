import pygame
import math
from robot import Robot
from map import World
from slam import GridBasedSLAM
from controller import Controller
from debug import DEBUG, DEBUG_OVERLAY, log_main, log_sensor

# Initialize pygame
pygame.init()

# Set up the display for a vertical aspect ratio (YouTube short format)
SCREEN_WIDTH, SCREEN_HEIGHT = 720, 900
UI_HEIGHT = 150  # Height reserved for the UI telemetry at the top
MAP_HEIGHT = SCREEN_HEIGHT - UI_HEIGHT  # The remaining height for the simulated map

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption('Simple SLAM Simulator')

# Font for telemetry data and other texts
font = pygame.font.SysFont("Arial", 24)
small_red_font = pygame.font.SysFont("Arial", 18)

# Create robot and world objects
robot = Robot([SCREEN_WIDTH // 2, UI_HEIGHT + MAP_HEIGHT // 2], 0)  # Start robot in the center of the map area
world = World()  # Define the world bounds
slam = GridBasedSLAM(80, 60)  # Start with 80x60 grid for the map
controller = Controller(robot=robot, slam=slam)  # Initialize the controller with the robot and SLAM system


# Colors
BACKGROUND_COLOR = (0, 0, 0)  # Black for map background
UI_BACKGROUND_COLOR = (50, 50, 50)  # Darker grey for UI section
TEXT_COLOR = (255, 255, 255)  # White text for telemetry
CLEAR_SPACE_COLOR = (128, 128, 128)  # Grey for clear space
OBSTACLE_FIRST_DETECTED_COLOR = (255, 255, 0)  # Yellow for first detection of obstacle
OBSTACLE_CONFIRMED_COLOR = (0, 255, 0)  # Green for confirmed obstacles
FRONTIER_COLOR = (64, 64, 64)  # Dark grey for frontier
ROBOT_COLOR = (0, 0, 255)  # Blue for robot
PATH_COLOR = (255, 255, 255)  # White for path
RED_TEXT_COLOR = (255, 0, 0)  # Red for the custom simulator text
GOAL_PATCH_COLOR = (0, 255, 255)  # Cyan to highlight the specific patch we are targeting
ASTAR_PATH_COLOR = (255, 165, 0)  # Orange for A* path dots

# Track robot path (real-world coordinates)
path_points = []

log_main(f"Starting SLAM simulator. DEBUG={DEBUG}, DEBUG_OVERLAY={DEBUG_OVERLAY}")

# Main game loop
running = True
frame_count = 0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Exit on pressing the Escape key or 'Q'
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                running = False

    # Get sensor data and update SLAM map
    # 1. Get the full 360 LiDAR scan
    lidar_data = robot.simulate_lidar(world.obstacles)
    
    # 2. Feed the full scan to SLAM
    slam.lidar_update(robot.position, lidar_data)
    
    # 3. Extract just the front ray (index 0) for the Controller's obstacle avoidance
    front_sensor_data = lidar_data[0][1] 
    
    # 4. Update the controller with the front ray
    controller.update(front_sensor_data, robot.position)

    if DEBUG and frame_count % 60 == 0:
        log_main(f"Frame {frame_count}: state={controller.state}, sensor={front_sensor_data:.1f}, path_steps={len(controller.current_path)}")
    frame_count += 1

    # Clear the screen with background color
    screen.fill(BACKGROUND_COLOR)

    # Draw the UI section at the top
    pygame.draw.rect(screen, UI_BACKGROUND_COLOR, pygame.Rect(0, 0, SCREEN_WIDTH, UI_HEIGHT))

    # Text for Telemetry display and top UI
    title_text = small_red_font.render("Totally Not Evit Robot Army - simple SLAM simulator", True, RED_TEXT_COLOR)
    robot_text = font.render("Robot sensor: 1 forward ultrasonic sensor", True, TEXT_COLOR)
    state_text = font.render(f"Controller state: {controller.state}", True, TEXT_COLOR)
    telemetry_text = font.render(f"Angle: {robot.angle:.2f}° | Position: ({int(robot.position[0])}, {int(robot.position[1])})", True, TEXT_COLOR)
    
    screen.blit(title_text, (10,10))
    screen.blit(robot_text, (10, 40))
    screen.blit(state_text, (10, 70))
    screen.blit(telemetry_text, (10, 100))

    # Centering: Keep the robot centered in the map area
    robot_screen_x = SCREEN_WIDTH // 2
    robot_screen_y = UI_HEIGHT + MAP_HEIGHT // 2

    # Calculate offset based on robot's position relative to the grid
    offset_x = SCREEN_WIDTH // 2 - robot.position[0]
    offset_y = UI_HEIGHT + MAP_HEIGHT // 2 - robot.position[1]

    # ==========================================
    # DEBUG OVERLAY (only shown when DEBUG and DEBUG_OVERLAY are True)
    # ==========================================
    if DEBUG and DEBUG_OVERLAY:
        debug_font = pygame.font.SysFont("Arial", 14)
        debug_color = (0, 255, 255)  # Cyan text for debug info

        # Highlight the A* waypoint the robot is currently targeting
        if controller.current_path:
            waypoint_grid = controller.current_path[0]
            wp_world_x = waypoint_grid[0] * 10 + 5
            wp_world_y = waypoint_grid[1] * 10 + 5
            wp_screen_x = wp_world_x + offset_x
            wp_screen_y = wp_world_y + offset_y
            if wp_screen_y > UI_HEIGHT:
                pygame.draw.circle(screen, (255, 255, 0), (int(wp_screen_x), int(wp_screen_y)), 6, 2)
                # Draw a line from robot to current waypoint
                pygame.draw.line(screen, (255, 255, 0), (robot_screen_x, robot_screen_y), (int(wp_screen_x), int(wp_screen_y)), 1)

        # Draw a debug info panel in the bottom-right corner of the map area
        panel_x = SCREEN_WIDTH - 310
        panel_y = SCREEN_HEIGHT - 140
        panel_w = 300
        panel_h = 130
        # Semi-transparent background
        panel_surface = pygame.Surface((panel_w, panel_h))
        panel_surface.set_alpha(200)
        panel_surface.fill((0, 0, 0))
        screen.blit(panel_surface, (panel_x, panel_y))
        pygame.draw.rect(screen, (0, 255, 255), pygame.Rect(panel_x, panel_y, panel_w, panel_h), 1)

        slam_map_dbg = slam.get_map()
        free_cells = int((slam_map_dbg < -1.0).sum())
        occ_cells = int((slam_map_dbg > 0.8).sum())
        unknown_cells = slam_map_dbg.size - free_cells - occ_cells
        grid_w, grid_h = slam.map_size

        debug_lines = [
            f"=== DEBUG PANEL ===",
            f"Front sensor reading: {front_sensor_data:.1f} px",
            f"SLAM grid: {grid_w} x {grid_h}",
            f"  Free cells:     {free_cells}",
            f"  Occupied cells: {occ_cells}",
            f"  Unknown cells:  {unknown_cells}",
            f"State: {controller.state}",
            f"Path length: {len(controller.current_path)}",
        ]
        for i, line in enumerate(debug_lines):
            txt = debug_font.render(line, True, debug_color)
            screen.blit(txt, (panel_x + 8, panel_y + 6 + i * 15))
    # ==========================================
    # END DEBUG OVERLAY
    # ==========================================

    # Track robot's path (append to the path array)
    path_points.append((robot.position[0], robot.position[1]))

    # Draw the SLAM map (only what the robot has detected)
    slam_map = slam.get_map()
    
    # Calculate the visible grid boundaries (25 cells in each direction from robot)
    start_x = max(0, int(robot.position[0] // 10) - 25)
    end_x = min(slam_map.shape[0], int(robot.position[0] // 10) + 25)
    start_y = max(0, int(robot.position[1] // 10) - 25)
    end_y = min(slam_map.shape[1], int(robot.position[1] // 10) + 25)

    for x in range(start_x, end_x):
        for y in range(start_y, end_y):
            # ... rest of your rendering code (rect_x, rect_y, etc) stays exactly the same
            rect_x = x * 10 + offset_x
            rect_y = y * 10 + offset_y

            if rect_y > UI_HEIGHT:
                val = slam_map[x, y]
                '''
                # ==========================================
                # DEBUG: LOG-ODDS HEATMAP RENDERING (ACTIVE)
                # ==========================================
                # 1. DEFINITELY FREE (val < -1.0)
                # Brighter Green = Higher Confidence it's empty
                if val < -1.0:
                    intensity = min(1.0, abs(val) / 5.0) # 5.0 is your L_CLAMP
                    color = (0, int(255 * intensity), 0)
                    pygame.draw.rect(screen, color, pygame.Rect(rect_x, rect_y, 10, 10))
                    
                # 2. UNKNOWN (val between -1.0 and 0.8)
                # Keep it black, but check for frontiers
                elif -1.0 <= val <= 0.8:
                    is_frontier = False
                    for dx, dy in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < slam_map.shape[0] and 0 <= ny < slam_map.shape[1]:
                            if slam_map[nx, ny] < -1.0: 
                                is_frontier = True
                                break
                                
                    if is_frontier:
                        pygame.draw.rect(screen, (0, 0, 255), pygame.Rect(rect_x, rect_y, 10, 10)) # Blue frontiers
                    else:
                        pygame.draw.rect(screen, (0, 0, 0), pygame.Rect(rect_x, rect_y, 10, 10))
                        
                # 3. OBSTACLES (val > 0.8)
                # Brighter Red/Magenta = Higher Confidence it's a wall
                elif val > 0.8:
                    intensity = min(1.0, val / 5.0) # 5.0 is your L_CLAMP
                    color = (int(255 * intensity), 0, int(100 * intensity)) # Reddish/Magenta
                    pygame.draw.rect(screen, color, pygame.Rect(rect_x, rect_y, 10, 10))
'''

                # ==========================================
                # OLD: STANDARD MAP RENDERING (COMMENTED OUT)
                # ==========================================
                
                if val < -1.0:  # Definitely Free
                    # Optional: Make darker grey if very confident, lighter if barely free
                    brightness = max(40, min(128, int(128 + (val * 20)))) 
                    pygame.draw.rect(screen, (brightness, brightness, brightness), pygame.Rect(rect_x, rect_y, 10, 10), 1)
                
                elif val > 2.0:  # Confirmed Obstacle (Green)
                    pygame.draw.rect(screen, OBSTACLE_CONFIRMED_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))
                
                elif val > 0.5:  # Tentative Obstacle (Yellow)
                    pygame.draw.rect(screen, OBSTACLE_FIRST_DETECTED_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))
                
                elif -0.8 <= val <= 0.8:  # Unknown (Check for frontier)
                    is_frontier = False
                    for dx, dy in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < slam_map.shape[0] and 0 <= ny < slam_map.shape[1]:
                            if slam_map[nx, ny] < -1.0: # Touching Free Space
                                is_frontier = True
                                break
                    if is_frontier:
                        pygame.draw.rect(screen, FRONTIER_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))
                

    # Draw the specific targeted goal frontier patch (Cyan)
    if controller.current_goal_patch:
        for world_x, world_y in controller.current_goal_patch:
            patch_rect_x = world_x + offset_x - 5
            patch_rect_y = world_y + offset_y - 5
            if patch_rect_y > UI_HEIGHT:
                pygame.draw.rect(screen, GOAL_PATCH_COLOR, pygame.Rect(int(patch_rect_x), int(patch_rect_y), 10, 10))

    # Draw A* path (Small Orange Dots)
    if controller.current_path:
        for grid_step in controller.current_path:
            # Convert grid step back to world coordinates to apply offset
            path_world_x = grid_step[0] * 10 + 5
            path_world_y = grid_step[1] * 10 + 5
            # Apply offset to get screen coordinates
            dot_screen_x = path_world_x + offset_x
            dot_screen_y = path_world_y + offset_y
            if dot_screen_y > UI_HEIGHT:
                pygame.draw.circle(screen, ASTAR_PATH_COLOR, (int(dot_screen_x), int(dot_screen_y)), 2) # Radius 2 for small dots

    # Draw robot's path using the real-world positions
    if len(path_points) > 1:
        # Keep path static relative to the world, rather than shifting with the robot
        transformed_path = [(x - robot.position[0] + robot_screen_x, y - robot.position[1] + robot_screen_y) for (x, y) in path_points]
        # Ensure the path does not draw in the telemetry area
        transformed_path = [(x, y) for (x, y) in transformed_path if y > UI_HEIGHT]
        pygame.draw.lines(screen, PATH_COLOR, False, transformed_path, 2)

    if controller.current_goal is not None:
        goal_screen_x = controller.current_goal[0] - robot.position[0] + robot_screen_x
        goal_screen_y = controller.current_goal[1] - robot.position[1] + robot_screen_y
        if goal_screen_y > UI_HEIGHT:
            pygame.draw.circle(screen, (0, 0, 255), (int(goal_screen_x), int(goal_screen_y)), 6)

    # Draw the robot at the center of the map area
    pygame.draw.circle(screen, ROBOT_COLOR, (robot_screen_x, robot_screen_y), robot.radius)
    line_length = robot.radius + 10
    direction_x = robot_screen_x + line_length * math.cos(math.radians(robot.angle))
    direction_y = robot_screen_y + line_length * math.sin(math.radians(robot.angle))
    pygame.draw.line(screen, (255, 255, 255), (robot_screen_x, robot_screen_y), (direction_x, direction_y), 2)

    # Update the display
    pygame.display.flip()

    # Small delay to control the frame rate
    pygame.time.delay(10)

log_main("Exiting SLAM simulator")
pygame.quit()