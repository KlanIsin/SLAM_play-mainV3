import heapq
from debug import log_pathfinder


class Pathfinder:
    def __init__(self, robot=None, slam=None):
        # You might initialize some variables here, or just use static methods
        pass

    def find_path(self, start_grid, goal_grid, map_grid):
        """This is the main method the Controller will call."""
        log_pathfinder(f"find_path: start={start_grid} goal={goal_grid} map_shape={map_grid.shape}")

        # 1. Initialize your Open and Closed lists
        open_list = []  
        closed_list = set() 

        # LOCAL dictionaries to track scores and parents
        origin_dict = {}
        g_scores = {} 

        # 2. Run the A* loop
        # 2.1 Setup the start node
        heapq.heappush(open_list, (0, start_grid))
        g_scores[start_grid] = 0
        origin_dict[start_grid] = None 

        nodes_explored = 0
        
        # 8-way directions
        directions_8 = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]

        while open_list:

            f, current_cell = heapq.heappop(open_list)
            nodes_explored += 1
            if nodes_explored > 5000:
                log_pathfinder("Path taking too long, aborting!")
                return []

            if current_cell in closed_list:
                continue

            if current_cell == goal_grid:
                path = self._reconstruct_path(origin_dict, current_cell)
                log_pathfinder(f"GOAL FOUND! Nodes explored: {nodes_explored}, path length: {len(path)}")
                return path

            closed_list.add(current_cell)

            for dx, dy in directions_8:
                neighbor = (current_cell[0] + dx, current_cell[1] + dy)

                # 2.5.1 Check if neighbor is within bounds
                if not (0 <= neighbor[0] < map_grid.shape[0] and 0 <= neighbor[1] < map_grid.shape[1]):
                    continue

                # 2.5.2 Check if neighbor is an obstacle
                if map_grid[neighbor] >= -0.8:
                    continue

                # CORNER CUTTING PREVENTION
                # If moving diagonally, check the two adjacent cells. 
                # If either is an obstacle, we can't cut diagonally!
                if dx != 0 and dy != 0:
                    cell1 = (current_cell[0] + dx, current_cell[1])
                    cell2 = (current_cell[0], current_cell[1] + dy)
                    if map_grid[cell1] >= -0.8 or map_grid[cell2] >= -0.8:
                        continue

                if neighbor in closed_list:
                    continue

                # 2.5.4 Calculate g, h, and f scores
                # Diagonal movement costs 1.414, straight costs 1.0
                base_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                
                # TURN COST LOGIC
                turn_penalty = 0
                parent_cell = origin_dict.get(current_cell)
                if parent_cell is not None:
                    dir1 = (current_cell[0] - parent_cell[0], current_cell[1] - parent_cell[1])
                    dir2 = (dx, dy)
                    if dir1 != dir2:
                        turn_penalty = 2.0  # Add penalty for changing direction

                tentative_g_score = g_scores[current_cell] + base_cost + turn_penalty
                h_score = self._calculate_heuristic(neighbor, goal_grid)
                f_score = tentative_g_score + h_score

                if neighbor not in g_scores or tentative_g_score < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g_score
                    origin_dict[neighbor] = current_cell
                    
                    heapq.heappush(open_list, (f_score, neighbor))

        log_pathfinder(f"NO PATH FOUND after exploring {nodes_explored} nodes")
        return []  

    def _calculate_heuristic(self, cell_a, cell_b):
        """Helper method to calculate the 'h' score using Octile Distance."""
        dx = abs(cell_a[0] - cell_b[0])
        dy = abs(cell_a[1] - cell_b[1])
        
        # Octile distance is the standard heuristic for 8-way grids
        # It accounts for diagonal steps being 1.414 and straight steps being 1.0
        return max(dx, dy) + (1.414 - 1) * min(dx, dy)

    def _reconstruct_path(self, origin_dict, current_cell):
        """Helper method to trace the path backwards once the goal is found."""
        # Read the dictionary to build the list of steps
        path_list = []
        while current_cell is not None:
            path_list.append(current_cell)
            current_cell = origin_dict[current_cell]
        path_list.reverse()  # Reverse the list to get the path from start to goal
        return path_list