import pygame
import math
from robot import Robot
from map import World
from slam import GridBasedSLAM
from controller import Controller

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

# Track robot path (real-world coordinates)
path_points = []

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Exit on pressing the Escape key or 'Q'
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                running = False

    # Get sensor data and update SLAM map
    sensor_data = robot.simulate_ultrasonic(world.obstacles)
    slam.sensor_update(robot.position, robot.angle, sensor_data)
    controller.update(sensor_data, robot.position)

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

    # Track robot's path (append to the path array)
    path_points.append((robot.position[0], robot.position[1]))

    # Centering: Keep the robot centered in the map area
    robot_screen_x = SCREEN_WIDTH // 2
    robot_screen_y = UI_HEIGHT + MAP_HEIGHT // 2

    # Calculate offset based on robot's position relative to the grid
    offset_x = SCREEN_WIDTH // 2 - robot.position[0]
    offset_y = UI_HEIGHT + MAP_HEIGHT // 2 - robot.position[1]

    # Draw the SLAM map (only what the robot has detected)
    slam_map = slam.get_map()
    for x in range(slam_map.shape[0]):
        for y in range(slam_map.shape[1]):
            rect_x = x * 10 + offset_x
            rect_y = y * 10 + offset_y

            if rect_y > UI_HEIGHT:  # Ensure map is drawn below the UI section
                if slam_map[x, y] == 1:  # Clear space (grey)
                    pygame.draw.rect(screen, CLEAR_SPACE_COLOR, pygame.Rect(rect_x, rect_y, 10, 10), 1)
                elif slam_map[x, y] == 2:  # First detected obstacle (yellow)
                    pygame.draw.rect(screen, OBSTACLE_FIRST_DETECTED_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))
                elif slam_map[x, y] == 3:  # Confirmed obstacle (green)
                    pygame.draw.rect(screen, OBSTACLE_CONFIRMED_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))
                elif slam_map[x, y] == 0:  # Unknown cell
                    # --- NEW FRONTIER DRAWING LOGIC ---
                    is_frontier = False
                    # Check 4 cardinal neighbors to see if this unknown cell touches a free cell
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < slam_map.shape[0] and 0 <= ny < slam_map.shape[1]:
                            if slam_map[nx, ny] == 1:
                                is_frontier = True
                                break
                    
                    if is_frontier:
                        pygame.draw.rect(screen, FRONTIER_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))
                    # --- END FRONTIER LOGIC ---

    # Draw the specific targeted goal frontier patch (Cyan)
    if controller.current_goal_patch:
        for world_x, world_y in controller.current_goal_patch:
            patch_rect_x = world_x + offset_x - 5
            patch_rect_y = world_y + offset_y - 5
            if patch_rect_y > UI_HEIGHT:
                pygame.draw.rect(screen, GOAL_PATCH_COLOR, pygame.Rect(int(patch_rect_x), int(patch_rect_y), 10, 10))

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

pygame.quit()
