import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import random
import time
import math
from heuristic import AStarPlanner

class DroneCity(gym.Env):
    def __init__(self):
        print("in init")
        super(DroneCity, self).__init__()
        
        self.physicsClient = p.connect(p.GUI)
        self.Z_AXIS = 1
        
        p.setAdditionalSearchPath('/Users/aldrinvrodrigues/Engineering/SEM-5/Kodikon-4.0')
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        p.setGravity(0, 0, 0)
        self.city_loaded = False 
        self.drone_loaded = False
        
        self.BLOCK_DIMENSIONS = [1, 1, 1]  # width, length, height for block
        self.CUBE_DIMENSIONS = [1, 1, 1]    # width, length, height for cube
        
        self.load_city_assets()
        self.drone_id = self.load_drone()
        
        self.observation_space = spaces.Box(
            low=np.array([-5, -5, -5, -5]),
            high=np.array([5, 5, 5, 5]),
            dtype=np.float32
        )
        
         # Action space (target x and y position changes)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        # Target coordinates for drone movement
        ##
        self.target_x, self.target_y = 1, 1
        self.planner = AStarPlanner(grid_size=(11, 11), obstacles=set())
        
        
        
    def is_overlapping(self, new_position, existing_positions, dimensions):
        half_width = dimensions[0] / 2
        half_length = dimensions[1] / 2
        new_min = [new_position[0] - half_width, new_position[1] - half_length]
        new_max = [new_position[0] + half_width, new_position[1] + half_length]

        for pos in existing_positions:
            pos_min = [pos[0] - half_width, pos[1] - half_length]
            pos_max = [pos[0] + half_width, pos[1] + half_length]
            # Check for overlap
            if (new_min[0] < pos_max[0] and new_max[0] > pos_min[0] and
                    new_min[1] < pos_max[1] and new_max[1] > pos_min[1]):
                return True
        return False

        
        
    def load_city_assets(self):
        print("in load city assets")
        if self.city_loaded:
            return 
        assets = {
            "plane": ([0, 0, -0.1], [0, 0, 0]),
            "stadium": ([0, 0, 0], [0, 0, 0]),
            "cube1": ([1, -4, 1], [0, 0, 0]),
            "cube2": ([3, 3, 1], [0, 0, 0]),
            "cube3": ([-4, 2, 1], [0, 0, 0]),
            "cube4": ([2, -2, 1], [0, 0, 0]),
            "cube5": ([0, 4, 1], [0, 0, 0]),
            "cube6": ([-3, -3, 1], [0, 0, 0]),
            "cube7": ([4, -1, 1], [0, 0, 0]),
            "cube8": ([-1, 0, 1], [0, 0, 0]),
            "cube9": ([5, 1, 1], [0, 0, 0]),
            "cube10": ([0, -3, 1], [0, 0, 0]),
            "cube11": ([-2, 3, 1], [0, 0, 0]),
            "cube12": ([1, 2, 1], [0, 0, 0]),
            "cube13": ([-5, -2, 1], [0, 0, 0]),
            "cube14": ([2, 0, 1], [0, 0, 0]),
            "cube15": ([-4, -1, 1], [0, 0, 0]),
            "cube16": ([3, -4, 1], [0, 0, 0]),
            "cube17": ([-3, 4, 1], [0, 0, 0]),
            "cube18": ([4, 0, 1], [0, 0, 0]),
            "cube19": ([-2, -4, 1], [0, 0, 0]),
            "cube20": ([5, -3, 1], [0, 0, 0]),
            "cube21": ([-1, 5, 1], [0, 0, 0]),
            "cube22": ([2, 5, 1], [0, 0, 0]),
            "cube23": ([-5, 1, 1], [0, 0, 0]),
            "cube24": ([0, -5, 1], [0, 0, 0]),
            "cube25": ([3, 1, 1], [0, 0, 0]),
            "cube26": ([-3, -1, 1], [0, 0, 0]),
            "cf2x" : ([0, 0, 0], [0, 0, 0]),
        }
        
        existing_positions = [assets[asset][0] for asset in assets]

        # for i in range(6, 15):
        #     # Find a valid position for the cube
        #     while True:
        #         random_position = [random.uniform(-5, 5), random.uniform(-5, 5), 1]
        #         random_position = [math.floor(random_position[0]), math.floor(random_position[1]), random_position[2]]
        #         if not self.is_overlapping(random_position, existing_positions, self.CUBE_DIMENSIONS):
        #             assets[f"cube{i}"] = (random_position, [0, 0, 0])
        #             existing_positions.append(random_position)
        #             break
        print("\n\n\n",existing_positions,"\n\n\n")
            # Load the assets
        for asset_name, (position, euler_orientation) in assets.items():
            orientation = p.getQuaternionFromEuler(euler_orientation)
            try:
                # if "block" in asset_name:
                #     p.loadURDF("block.urdf", basePosition=position, baseOrientation=orientation)
                if "cube" in asset_name:
                    # p.loadURDF("cube.urdf", basePosition=position, baseOrientation=orientation)
                    p.loadURDF("Drone Model + Script/rectangle.urdf", basePosition=position, baseOrientation=orientation)
                elif "cf2x" in asset_name:
                    pass
                    # drone_id = p.loadURDF("Drone Model + Script/cf2x.urdf", [0, 0, 0.1], baseOrientation=orientation)
                    # check_keyboard_and_control(drone_id)
                else:
                    p.loadURDF(f"{asset_name}.urdf", basePosition=position, baseOrientation=orientation)
            except:
                print(f"Failed to load asset: {asset_name}")
                obj_ids = p.loadSDF(f"{asset_name}.sdf")
                
                # If SDF loaded successfully, set positions for each object
                for obj_id in obj_ids:
                    p.resetBasePositionAndOrientation(obj_id, position, orientation)
        self.city_loaded = True  
    
    def load_drone(self):
        print("in load drone")
        if self.drone_loaded:
            return 
        drone_id = p.loadURDF("Drone Model + Script/cf2x.urdf", [0, 0, self.Z_AXIS], baseOrientation=[0, 0, 0, 1])
        self.drone_loaded = True
        return drone_id
    
    
    '''def get_next_coordinates(self):
        print("in get next coordinates")
        # Returns random coordinates as placeholders for the next target
        return np.random.uniform(-5, 5), np.random.uniform(-5, 5)'''
    
    def reset(self, seed=None, options=None):
        print("in reset")
        # Reset the environment and return initial observation
        
        if hasattr(self, 'drone_id') and self.drone_id is not None:
            orientation = p.getQuaternionFromEuler([0, 0, 0])
            p.resetBasePositionAndOrientation(self.drone_id, [0, 0, self.Z_AXIS], orientation)
        else:
            self.drone_id = self.load_drone()
            
        self.load_city_assets()
            
        self.drone_id = self.load_drone()
        
        # Randomly generate a new target position
        ##
        # self.target_x, self.target_y = self.get_next_coordinates()
        
        # Return initial observation (drone position and target coordinates)
        drone_pos = p.getBasePositionAndOrientation(self.drone_id)[0]
        return np.array([drone_pos[0], drone_pos[1], self.target_x, self.target_y], dtype=np.float32), {}
    
    '''def step(self, action):
        print("in step")
        # Take action and simulate environment
        target_x = np.clip(self.target_x + action[0], -5, 5)
        target_y = np.clip(self.target_y + action[1], -5, 5)
        
        self.move_drone_to_target(self.drone_id, target_x, target_y)
        
        # Get updated drone position
        drone_pos = p.getBasePositionAndOrientation(self.drone_id)[0]
        distance = np.sqrt((target_x - drone_pos[0])**2 + (target_y - drone_pos[1])**2)
        
        # Define a reward
        reward = -distance
        if distance < 0.1:  # Define when to end the episode
            terminated = True  
        else:   
            terminated = False 
        
        # Construct the observation
        obs = np.array([drone_pos[0], drone_pos[1], self.target_x, self.target_y], dtype=np.float32)
        
        if terminated:
            # Optionally reset the target position for the next episode
            self.target_x = np.random.uniform(-5, 5)
            self.target_y = np.random.uniform(-5, 5)

        return obs, reward, terminated, False, {}  # Return obs, reward, done, truncated, info'''
        
    def step(self, action=None):
        print("in step")

        # Get the drone's current position
        drone_pos = p.getBasePositionAndOrientation(self.drone_id)[0]

        # Run A* and get the path
        start = (int(drone_pos[0]), int(drone_pos[1]))
        goal = (int(self.target_x), int(self.target_y))
        path = self.planner.find_path(start, goal)

        # If a path exists, move to the next position on it
        if path and len(path) > 1:
            next_step = path[1]  # Skip the start position
            self.move_drone_to_target(self.drone_id, next_step[0], next_step[1])

        # Define reward and check for termination
        distance = np.linalg.norm(np.array([self.target_x, self.target_y]) - np.array(drone_pos[:2]))
        reward = -distance
        terminated = distance < 0.1

        obs = np.array([drone_pos[0], drone_pos[1], self.target_x, self.target_y], dtype=np.float32)
        return obs, reward, terminated, False, {}

    
    
    def move_drone_to_target(self, target_x, target_y, speed=0.1):
        current_position = p.getBasePositionAndOrientation(self.drone_id)[0]
        current_x, current_y = current_position[0], current_position[1]

        # Move to target coordinates smoothly
        while True:
            # Calculate the distance to the target
            distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

            # If we are close enough to the target, break
            if distance < 0.1:
                break

            # Calculate the new position
            current_x += speed * (target_x - current_x) / distance
            current_y += speed * (target_y - current_y) / distance

            # Update the drone's position
            p.resetBasePositionAndOrientation(self.drone_id, [current_x, current_y, self.Z_AXIS], [0, 0, 0, 1])
            
            # Step the simulation
            p.stepSimulation()
            time.sleep(1./240.)
            
    def render(self, mode="human"):
        pass

    def close(self):
        p.disconnect()
        
    def navigate_to_goal(self, start, goal):
        grid_size = (11, 11)
        obstacles = {(int(x), int(y)) for x, y, _ in self.existing_positions}
        planner = AStarPlanner(grid_size=grid_size, obstacles=obstacles)

        path = planner.find_path(start, goal)
        
        if path:
            print("Path found:", path)
            for step in path:
                x, y = step
                self.move_drone_to_target(self.drone_id, x, y)
        else:
            print("No path found.")
    


def main():
    simulation = DroneCity()
    while True:
        action = None  # Define how you want to generate actions, e.g., random or based on RL policy
        obs, reward, done, _, _ = simulation.step(action)
        if done:
            break

if __name__ == "__main__":
    main() 

