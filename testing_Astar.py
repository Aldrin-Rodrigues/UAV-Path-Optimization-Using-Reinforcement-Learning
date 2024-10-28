import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import random
import time
import math
from heuristic import AStarPlanner

assets = {
    "plane": ([0, 0, -0.1], [0, 0, 0]),
        "stadium": ([0, 0, 0], [0, 0, 0]),
        "cube1": ([1.5, -3.5, 1], [0, 0, 0]),
        "cube2": ([3.5, 3.5, 1], [0, 0, 0]),
        "cube4": ([2.5, -2.5, 1], [0, 0, 0]),
        "cube5": ([0.5, 3.5, 1], [0, 0, 0]),
        "cube6": ([-3.5, -2.5, 1], [0, 0, 0]),
        "cube8": ([-1.5, 0.5, 1], [0, 0, 0]),
        "cube10": ([0.5, -3.5, 1], [0, 0, 0]),
        "cube11": ([-2.5, 2.5, 1], [0, 0, 0]),
        "cube12": ([0.5, 2.5, 1], [0, 0, 0]),
        "cube14": ([2.5, 0.5, 1], [0, 0, 0]),
        "cube15": ([-4.5, -1.5, 1], [0, 0, 0]),
        "cube16": ([3.5, -4.5, 1], [0, 0, 0]),
        "cube18": ([4.5, 0.5, 1], [0, 0, 0]),
        "cube19": ([-2.5, -4.5, 1], [0, 0, 0]),
        "cube21": ([-1.5, 5.5, 1], [0, 0, 0]),
        "cube23": ([-5, 1, 1], [0, 0, 0]),
        "cube25": ([3.5, 1.5, 1], [0, 0, 0]),
        "cube26": ([-3.5, -1.5, 1], [0, 0, 0]),
        "cf2x" : ([0, 0, 0], [0, 0, 0]),
    }


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
        
        self.BLOCK_DIMENSIONS = [1, 1, 1]
        self.CUBE_DIMENSIONS = [1, 1, 1]
        
        # Load assets and create obstacle set
        self.load_city_assets()
        self.drone_id = self.load_drone()
        
        # Create obstacle set from assets
        self.obstacles = set()
        for asset_name, (position, _) in assets.items():
            if "cube" in asset_name:
                self.obstacles.add((int(round(position[0])), int(round(position[1]))))
        
        print(f"Obstacles: {self.obstacles}")  # Debug print
        
        # Initialize target coordinates
        self.target_x = 2
        self.target_y = 2
        
        # Initialize planner with obstacles
        self.planner = AStarPlanner(grid_size=(11, 11), obstacles=self.obstacles)
    def load_city_assets(self):
        print("in load city assets")
        if self.city_loaded:
            return 
        assets = {
            "plane": ([0, 0, -0.1], [0, 0, 0]),
            "stadium": ([0, 0, 0], [0, 0, 0]),
            "cube1": ([1.5, -3.5, 1], [0, 0, 0]),
            "cube2": ([3.5, 3.5, 1], [0, 0, 0]),
            "cube4": ([2.5, -2.5, 1], [0, 0, 0]),
            "cube5": ([0.5, 3.5, 1], [0, 0, 0]),
            "cube6": ([-3.5, -2.5, 1], [0, 0, 0]),
            "cube8": ([-1.5, 0.5, 1], [0, 0, 0]),
            "cube10": ([0.5, -3.5, 1], [0, 0, 0]),
            "cube11": ([-2.5, 2.5, 1], [0, 0, 0]),
            "cube12": ([0.5, 2.5, 1], [0, 0, 0]),
            "cube14": ([2.5, 0.5, 1], [0, 0, 0]),
            "cube15": ([-4.5, -1.5, 1], [0, 0, 0]),
            "cube16": ([3.5, -4.5, 1], [0, 0, 0]),
            "cube18": ([4.5, 0.5, 1], [0, 0, 0]),
            "cube19": ([-2.5, -4.5, 1], [0, 0, 0]),
            "cube21": ([-1.5, 5.5, 1], [0, 0, 0]),
            "cube23": ([-5, 1, 1], [0, 0, 0]),
            "cube25": ([3.5, 1.5, 1], [0, 0, 0]),
            "cube26": ([-3.5, -1.5, 1], [0, 0, 0]),
            "cf2x" : ([0, 0, 0], [0, 0, 0]),
        }
        
        existing_positions = [assets[asset][0] for asset in assets]
        
        print("\n\n\n",existing_positions,"\n\n\n")
            # Load the assets
        for asset_name, (position, euler_orientation) in assets.items():
            orientation = p.getQuaternionFromEuler(euler_orientation)
            try:
                if "cube" in asset_name:
                    p.loadURDF("Drone Model + Script/rectangle.urdf", basePosition=position, baseOrientation=orientation)
                elif "cf2x" in asset_name:
                    pass
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
    
    def reset(self, seed=None, options=None):
        print("Resetting environment")
        # Reset drone position
        if self.drone_id is not None:
            p.resetBasePositionAndOrientation(
                self.drone_id, 
                [0, 0, self.Z_AXIS],
                [0, 0, 0, 1]
            )
        
        # Get initial drone position
        drone_pos = p.getBasePositionAndOrientation(self.drone_id)[0]
        
        # Return initial observation
        observation = np.array([
            drone_pos[0],
            drone_pos[1],
            self.target_x,
            self.target_y
        ], dtype=np.float32)
        
        return observation, {}
        
    def step(self, action=None):
        # Get current drone position
        drone_pos = p.getBasePositionAndOrientation(self.drone_id)[0]
        current_pos = (drone_pos[0], drone_pos[1])
        
        # Convert current position and target to grid coordinates
        start = (int(round(current_pos[0])), int(round(current_pos[1])))
        goal = (int(round(self.target_x)), int(round(self.target_y)))
        
        print(f"Current position: {start}, Target: {goal}")  # Debug print
        
        # Find path using A*
        path = self.planner.find_path(start, goal)
        print(f"Found path: {path}")  # Debug print
        
        if path and len(path) > 1:
            # Move to next position in path
            next_pos = path[1]  # Get next position after current position
            print(f"Moving to next position: {next_pos}")  # Debug print
            self.move_drone_to_target(next_pos[0], next_pos[1])
        else:
            print("No valid path found or already at target")
        
        # Get new drone position after movement
        new_drone_pos = p.getBasePositionAndOrientation(self.drone_id)[0]
        
        # Calculate reward and check for termination
        distance = np.sqrt((self.target_x - new_drone_pos[0])**2 + 
                        (self.target_y - new_drone_pos[1])**2)
        reward = -distance
        terminated = distance < 0.1
        
        # Create observation
        obs = np.array([new_drone_pos[0], new_drone_pos[1], 
                    self.target_x, self.target_y], dtype=np.float32)
        
        return obs, reward, terminated, False, {}

def main():
    env = DroneCity()
    
    # Reset the environment
    obs, _ = env.reset()
    
    # Set target position
    env.target_x = 4
    env.target_y = 6
    print(f"Target position set to: ({env.target_x}, {env.target_y})")
    
    done = False
    steps = 0
    max_steps = 1000  # Add maximum steps to prevent infinite loops
    
    while not done and steps < max_steps:
        obs, reward, done, _, _ = env.step()
        print(f"Step {steps}: Position = ({obs[0]:.2f}, {obs[1]:.2f}), Distance to target = {-reward:.2f}")
        time.sleep(0.01)
        steps += 1
    
    if done:
        print("Target reached!")
    elif steps >= max_steps:
        print("Maximum steps reached without finding target")
    
    env.close()

if __name__ == "__main__":
    main()




