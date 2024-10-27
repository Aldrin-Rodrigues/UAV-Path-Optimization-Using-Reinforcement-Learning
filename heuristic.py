import numpy as np
import heapq

class AStarPlanner:
    def __init__(self, grid_size, obstacles):
        self.grid_size = grid_size
        self.obstacles = obstacles
        self.open_list = []
        self.closed_list = set()
        self.came_from = {}
        self.g_score = {}
        self.f_score = {}
    
    def heuristic(self, start, goal):
        # Euclidean distance heuristic
        return np.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)

    def get_neighbors(self, node):
        neighbors = []
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # Right, Left, Up, Down
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if (-self.grid_size[0] <= neighbor[0] < self.grid_size[0]) and (-self.grid_size[1] <= neighbor[1] < self.grid_size[1]):
                if neighbor not in self.obstacles:  # Avoid obstacles
                    neighbors.append(neighbor)
        return neighbors

    def reconstruct_path(self, current):
        path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def find_path(self, start, goal):
        self.open_list = []
        self.closed_list = set()
        self.came_from = {}
        
        self.g_score = {start: 0}
        self.f_score = {start: self.heuristic(start, goal)}
        
        heapq.heappush(self.open_list, (self.f_score[start], start))
        
        while self.open_list:
            _, current = heapq.heappop(self.open_list)
            
            if current == goal:
                return self.reconstruct_path(current)
            
            self.closed_list.add(current)
            
            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_list:
                    continue
                
                tentative_g_score = self.g_score[current] + 1  # Cost to move to neighbor is 1 unit
                
                if neighbor not in self.g_score or tentative_g_score < self.g_score[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = tentative_g_score
                    self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    heapq.heappush(self.open_list, (self.f_score[neighbor], neighbor))
        
        return None  # No path found


# import heapq
# import numpy as np
# from typing import List, Set, Tuple, Dict, Optional

# class AStarPlanner3D:
#     def __init__(self, grid_size: Tuple[int, int], obstacles: Set[Tuple[int, int]]):
#         """
#         Initialize the A* pathfinding planner.
        
#         Args:
#             grid_size (tuple): The size of the grid (width, height)
#             obstacles (set): Set of obstacle coordinates (x, y)
#         """
#         self.width, self.height = grid_size
#         self.obstacles = obstacles
        
#         # Possible movements: (x, y) - 8-directional movement
#         self.movements = [
#             (1, 0),   # right
#             (-1, 0),  # left
#             (0, 1),   # up
#             (0, -1),  # down
#             (1, 1),   # up-right
#             (-1, 1),  # up-left
#             (1, -1),  # down-right
#             (-1, -1)  # down-left
#         ]

#     def is_valid_position(self, x: int, y: int) -> bool:
#         """
#         Check if a position is valid (within bounds and not an obstacle).
        
#         Args:
#             x (int): X coordinate
#             y (int): Y coordinate
            
#         Returns:
#             bool: True if position is valid, False otherwise
#         """
#         # Check boundaries
#         if x < -self.width//2 or x > self.width//2 or y < -self.height//2 or y > self.height//2:
#             return False
        
#         # Check if position is obstacle
#         if (x, y) in self.obstacles:
#             return False
            
#         return True

#     def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
#         """
#         Get valid neighboring positions for the current position.
        
#         Args:
#             current (tuple): Current position (x, y)
            
#         Returns:
#             list: List of valid neighboring positions
#         """
#         neighbors = []
#         x, y = current
        
#         for dx, dy in self.movements:
#             new_x, new_y = x + dx, y + dy
#             if self.is_valid_position(new_x, new_y):
#                 neighbors.append((new_x, new_y))
                
#         return neighbors

#     def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
#         """
#         Calculate the heuristic (estimated distance) between two points.
#         Using Euclidean distance as the heuristic.
        
#         Args:
#             a (tuple): Start position (x, y)
#             b (tuple): End position (x, y)
            
#         Returns:
#             float: Estimated distance between points
#         """
#         return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

#     def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
#         """
#         Find a path from start to goal using A* algorithm.
        
#         Args:
#             start (tuple): Start position (x, y)
#             goal (tuple): Goal position (x, y)
            
#         Returns:
#             list: List of positions forming the path, or None if no path is found
#         """
#         if not self.is_valid_position(*start) or not self.is_valid_position(*goal):
#             print("Invalid start or goal position")
#             return None

#         # Initialize the open and closed sets
#         open_set = []
#         closed_set = set()
        
#         # Dictionary to store the best previous node for each node
#         came_from = {}
        
#         # Dictionary to store g_scores (cost from start to node)
#         g_score = {start: 0}
        
#         # Dictionary to store f_scores (estimated total cost from start to goal through node)
#         f_score = {start: self.heuristic(start, goal)}
        
#         # Add start node to open set
#         heapq.heappush(open_set, (f_score[start], start))
        
#         while open_set:
#             # Get node with lowest f_score
#             current = heapq.heappop(open_set)[1]
            
#             # If we reached the goal, reconstruct and return the path
#             if current == goal:
#                 path = []
#                 while current in came_from:
#                     path.append(current)
#                     current = came_from[current]
#                 path.append(start)
#                 path.reverse()
#                 return path
            
#             # Add current node to closed set
#             closed_set.add(current)
            
#             # Check all neighbors
#             for neighbor in self.get_neighbors(current):
#                 if neighbor in closed_set:
#                     continue
                
#                 # Calculate tentative g_score for this neighbor
#                 tentative_g_score = g_score[current] + 1  # Using 1 as the cost between adjacent nodes
                
#                 # If we haven't seen this neighbor before or if this path to neighbor is better
#                 if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                     # Update the best path
#                     came_from[neighbor] = current
#                     g_score[neighbor] = tentative_g_score
#                     f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    
#                     # Add to open set if not already there
#                     if neighbor not in [item[1] for item in open_set]:
#                         heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
#         # If we get here, no path was found
#         print("No path found")
#         return None

#     def smooth_path(self, path: List[Tuple[int, int]], smooth_factor: float = 0.5) -> List[Tuple[float, float]]:
#         """
#         Smooth the path using path smoothing algorithm.
        
#         Args:
#             path (list): Original path consisting of waypoints
#             smooth_factor (float): Factor determining smoothing strength (0 to 1)
            
#         Returns:
#             list: Smoothed path
#         """
#         if not path or len(path) <= 2:
#             return path

#         smooth_path = [[float(x), float(y)] for x, y in path]
        
#         change = True
#         while change:
#             change = False
#             for i in range(1, len(smooth_path) - 1):
#                 for j in range(2):  # For both x and y coordinates
#                     old = smooth_path[i][j]
#                     smooth_path[i][j] += smooth_factor * (
#                         smooth_path[i-1][j] + smooth_path[i+1][j] - 2.0 * smooth_path[i][j]
#                     )
#                     if abs(old - smooth_path[i][j]) > 0.01:
#                         change = True

#         return [(x, y) for x, y in smooth_path]