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
        
        return None