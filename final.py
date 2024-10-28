import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import random
import time
import math
import os
from collections import deque

class DroneCity(gym.Env):
    def __init__(self,render_mode=None):
        # print("in init")
        super(DroneCity, self).__init__()
        
        self.physicsClient = p.connect(p.GUI)
        self.Z_AXIS = 1
        self.Battery = 100
        self.episode_count = 0
        self.episode_threshold = 690
        self.render_mode = render_mode
        
        self.total_reward = 0  # Initialize total_reward
        self.prev_rewards = 0
        
        #current wokring directory
        urdf_path = os.path.dirname(__file__)
        
        p.setAdditionalSearchPath(urdf_path)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        p.setGravity(0, 0, -9.81)
        self.city_loaded = False 
        self.drone_loaded = False
        
        
        self.CUBE_DIMENSIONS = [1, 1, 1]    # width, length, height for cube
        
        self.load_city_assets()
        self.drone_id = self.load_drone()
        
        
        
        
        self.mass = p.getDynamicsInfo(self.drone_id, -1)[0]
        
        self.rotor_indices = [0, 1, 2, 3]  # Adjust according to your URDF

        # Thrust needed per rotor to hover
        self.hover_thrust = self.mass * 9.81 / len(self.rotor_indices)
        self.thrust_z = self.hover_thrust
        self.rotor_speeds = [15,15,15,15]
        
        self.prev_distance = 0
        
        

        
        self.action_space = spaces.Discrete(6)

        self.destination_x , self.destination_y, self.destination_z = self.load_destinations()
        
        
        
        self.observation_space = spaces.Box(
            low=-5,  # x, y, orientation, min distance
            high=5,
            shape=(1005,),
            dtype=np.float32
        )


        # self.obs = self.current_state_space()

        
        
        
        
        
        
    def current_state_space(self):
        # Get drone position and orientatino in euler form
        drone_pos, drone_orientation = p.getBasePositionAndOrientation(self.drone_id)[0], p.getBasePositionAndOrientation(self.drone_id)[1]
        drone_orientation = p.getEulerFromQuaternion(drone_orientation)
        drone_x, drone_y, drone_z = drone_pos[0], drone_pos[1], drone_pos[2]
        
        # Calculate distance to target
        distance_to_target = np.sqrt((self.destination_x - drone_x)**2 + (self.destination_y - drone_y)**2 + (self.destination_z - drone_z)**2)
        
        # obs = [drone_x, drone_y, drone_z, drone_orientation[2], distance_to_target]
        obs = [drone_x, drone_y, drone_z,drone_orientation[2], distance_to_target]
        # print(drone_orientation[2])
        return obs
            

        
        
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

     
     
     
    def load_destinations(self):
        # print("in load destination")
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        destination_id = p.loadURDF("sphere_transparent.urdf", [4.9, 0, 1], orientation, useFixedBase=True)
        dest_x, dest_y, dest_z = p.getBasePositionAndOrientation(destination_id)[0]
        return dest_x, dest_y, dest_z
        
        
    def load_city_assets(self):
        # print("in load city assets")
        
        if self.city_loaded:
            return 
        assets = {
            "plane": ([0, 0, -0.1], [0, 0, 0]),
            "stadium": ([0, 0, 0], [0, 0, 0]),
            "cube1": ([1, -4, 0.8], [0, 0, 0]),
            "cube2": ([3, 3, 0.8], [0, 0, 0]),
            "cube3": ([-4, 2, 0.8], [0, 0, 0]),
            "cube4": ([2, -2, 0.8], [0, 0, 0]),
            "cube5": ([0, 4, 0.8], [0, 0, 0]),
            "cube6": ([-3, -3, 0.8], [0, 0, 0]),
            "cube7": ([4, -1, 0.8], [0, 0, 0]),
            "cube8": ([-1, 0, 0.8], [0, 0, 0]),
            # "cube9": ([5, 1, 0.8], [0, 0, 0]),
            "cube10": ([0, -3, 0.8], [0, 0, 0]),
            "cube11": ([-2, 3, 0.8], [0, 0, 0]),
            "cube12": ([1, 2, 0.8], [0, 0, 0]),
            "cube13": ([-5, -2, 0.8], [0, 0, 0]),
            "cube14": ([2, 0, 0.8], [0, 0, 0]),
            "cube15": ([-4, -1, 0.8], [0, 0, 0]),
            "cube16": ([3, -4, 0.8], [0, 0, 0]),
            "cube17": ([-3, 4, 0.8], [0, 0, 0]),
            # "cube18": ([4, 0, 0.8], [0, 0, 0]),
            "cube19": ([-2, -4, 0.8], [0, 0, 0]),
            # "cube20": ([5, -3, 0.8], [0, 0, 0]),
            "cube21": ([-1, 5, 0.8], [0, 0, 0]),
            "cube22": ([2, 5, 0.8], [0, 0, 0]),
            "cube23": ([-5, 1, 0.8], [0, 0, 0]),
            "cube24": ([0, -5, 0.8], [0, 0, 0]),
            "cube25": ([3, 1, 0.8], [0, 0, 0]),
            "cube26": ([-3, -1, 0.8], [0, 0, 0]),
            "cf2x" : ([0, 0, 0], [0, 0, 0]),
        }
        
        
        # existing_positions = [assets[asset][0] for asset in assets]
        # if self.episode_count % self.episode_threshold == 0:
        #     for i in range(6, 15):
        #         # Find a valid position for the cube
        #         while True:
        #             random_position = [random.uniform(-5, 5), random.uniform(-5, 5), 1]
        #             random_postition = [math.ceil(random_position[0]), math.ceil(random_position[1]), 1]
        #             if not self.is_overlapping(random_position, existing_positions, self.CUBE_DIMENSIONS):
        #                 assets[f"cube{i}"] = (random_position, [0, 0, 0])
        #                 existing_positions.append(random_position)
        #                 break
    
                    
        # Load the assets
        for asset_name, (position, euler_orientation) in assets.items():
            orientation = p.getQuaternionFromEuler(euler_orientation)
            try:
                # if "block" in asset_name:
                #     p.loadURDF("block.urdf", basePosition=position, baseOrientation=orientation)
                if "cube" in asset_name:
                    p.loadURDF("Drone Model + Script/rectangle.urdf", basePosition=position, baseOrientation=orientation, useFixedBase=True)
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
                    p.changeDynamics(obj_id, -1, mass=0)
        self.city_loaded = True  
    
    def load_drone(self):
        if self.drone_loaded:
            return
        # print("in load drone")
        drone_id = p.loadURDF("Drone Model + Script/cf2x.urdf", [-4.8, 0, 0.7], baseOrientation=[0, 0, 0, 1])
        
        
        self.drone_loaded = True
        # print("drone loaded:" , drone_id)
        return drone_id
    

    
    
    
    def reset(self, seed=None, options=None):
        # print("in reset")
        self.Battery = 100
        self.prev_rewards=0
        self.done = False
        
        
        self.prev_rewards = 0
        self.total_reward = 0
        
        self.prev_action = deque(maxlen=1000)
        
        for i in range(0,1000):
            self.prev_action.append(-1)

        # Reset the environment and return initial observation
        
        # # Handle the seed for reproducibility
        # super().reset(seed=seed)
        # np.random.seed(seed)
        
        
        if hasattr(self, 'drone_id') and self.drone_id is not None:
            orientation = p.getQuaternionFromEuler([0, 0, 0])
            p.resetBasePositionAndOrientation(self.drone_id, [-4.8, 0, 0.7], orientation)
        else:
            self.drone_id = self.load_drone()
            
        self.load_city_assets()
        # print("reset", self.drone_id)
            
        # self.drone_id = self.load_drone()
        
        
        # Return initial observation (drone position and target coordinates)
        obs = self.current_state_space()
        
        info = {}
        return (obs + list(self.prev_action)), info
        # return np.array(obs), info
    
        
    
    
    
    
    def step(self, action):
        # print("in step")
        self.prev_action.append(action)

        velocity = p.getBaseVelocity(self.drone_id)[0]
        # time.sleep(1)
        
        # print(action)
        # idx = 0
    
        
        print(action)
        
        if action == 0:
            for i in self.rotor_indices:
                # print(i)
                thrust = self.rotor_speeds[i]
                if i in [0,2]:
                    p.applyExternalForce(self.drone_id, i, [thrust, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)
                    # print("Drone id",self.drone_id)
                else:
                    p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)

        elif action == 1:
               
            for i in self.rotor_indices:
                if i in [0,1]:
                    p.applyExternalForce(self.drone_id, i, [15, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)
                else:
                    p.applyExternalForce(self.drone_id, i, [-15, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)
                    
                    
        elif action == 2:
            # idx += 1
            for i in self.rotor_indices:
                if i in [0,1]:
                    p.applyExternalForce(self.drone_id, i, [-15, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)
                else:
                    p.applyExternalForce(self.drone_id, i, [15, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)
                    

                    
                    
        elif action == 3:
            # idx += 1
            for i in self.rotor_indices:
                p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z+4], [0,0,0], p.LINK_FRAME)
                
                
        elif action == 4:
            # idx += 1
            for i in self.rotor_indices:
                p.applyExternalForce(self.drone_id, i, [0, 0, 4], [0,0,0], p.LINK_FRAME)
                
                
        elif action == 5:
            # pass
            # idx += 1
            for i in self.rotor_indices:
                p.applyExternalForce(self.drone_id, i, [0,0,self.thrust_z], [0,0,0], p.LINK_FRAME)
            p.resetBaseVelocity(self.drone_id, linearVelocity = [velocity[0]*0.5, velocity[1]*0.5, velocity[2]*0.5], angularVelocity = [0, 0, 0])
        
        
        obs = self.current_state_space()
        # print(obs)
        observation = np.array(obs + list(self.prev_action))
        # observation = np.array(obs)
        
        
        
        # Define a reward
        # self.total_reward = -obs[4]
        if abs(obs[4]) < 0.5:  # Define when to end the episode
            self.done = True
            self.total_reward = 500

        else:
            self.done = False
            

        collision_points = p.getContactPoints(bodyA=self.drone_id)  # Check for contacts involving the drone
        if collision_points:
            self.total_reward -= 10  # Optionally add a penalty for collision
            # Collision detected
            self.done = True
        
        if obs[2] > 2.8 or obs[2]<0.1:
            self.total_reward -= 5
            
            self.done = True
            
        if obs[2]<0.3:
            self.total_reward -= 15
            for i in self.rotor_indices:
                p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z+2], [0,0,0], p.LINK_FRAME)
                
        if obs[2]>2.3:
            self.total_reward -= 2
        
        if obs[2]>2.6:
            self.total_reward -= 15
            for i in self.rotor_indices:
                p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z-2], [0,0,0], p.LINK_FRAME)
            
            


        
        if obs[4] >= math.sqrt(((-5 - self.destination_x)**2 + (0 - self.destination_y)**2 + (1 - self.destination_z)**2)):
            self.total_reward -= 1
        
        if np.random.random() < 0.01:
            self.prev_distance = obs[4]
            
        if self.prev_distance < obs[4]:
            print("me")
            self.total_reward -= 0.05
            for i in self.rotor_indices:
                if i in [0,1]:
                    p.applyExternalForce(self.drone_id, i, [-15, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)
                else:
                    p.applyExternalForce(self.drone_id, i, [15, 0, self.thrust_z], [0,0,0], p.LINK_FRAME)
            
        else:
            self.total_reward += (0.001)
            

        print(self.total_reward)
            
        
        
        
        self.reward = self.total_reward - self.prev_rewards
        self.prev_rewards = self.total_reward
        p.stepSimulation()
        return observation, self.reward, self.done, False, {}  # Return obs, reward, done, truncated, info
    
    # def step(self, action):
    #     # Store the current distance to the target before taking the action
    #     prev_distance_to_target = self.current_state_space()[4]
        
    #     # Apply the action
    #     self.prev_action.append(action)
    #     velocity = p.getBaseVelocity(self.drone_id)[0]
        
    #     print(f"Action: {action}\n")

    #     # Action handling (rotor forces and adjustments)
    #     if action == 0:
    #         for i in self.rotor_indices:
    #             thrust = self.rotor_speeds[i]
    #             if i in [0, 2]:
    #                 p.applyExternalForce(self.drone_id, i, [thrust, 0, self.thrust_z], [0, 0, 0], p.LINK_FRAME)
    #             else:
    #                 p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z], [0, 0, 0], p.LINK_FRAME)

    #     elif action == 1:
    #         for i in self.rotor_indices:
    #             if i in [0, 1]:
    #                 p.applyExternalForce(self.drone_id, i, [10, 0, self.thrust_z], [0, 0, 0], p.LINK_FRAME)
    #             else:
    #                 p.applyExternalForce(self.drone_id, i, [-10, 0, self.thrust_z], [0, 0, 0], p.LINK_FRAME)

    #     elif action == 2:
    #         for i in self.rotor_indices:
    #             if i in [0, 1]:
    #                 p.applyExternalForce(self.drone_id, i, [-10, 0, self.thrust_z], [0, 0, 0], p.LINK_FRAME)
    #             else:
    #                 p.applyExternalForce(self.drone_id, i, [10, 0, self.thrust_z], [0, 0, 0], p.LINK_FRAME)

    #     elif action == 3:
    #         for i in self.rotor_indices:
    #             p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z + 5], [0, 0, 0], p.LINK_FRAME)

    #     elif action == 4:
    #         for i in self.rotor_indices:
    #             p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z-1], [0, 0, 0], p.LINK_FRAME)

    #     elif action == 5:
    #         for i in self.rotor_indices:
    #             p.applyExternalForce(self.drone_id, i, [0, 0, self.thrust_z], [0, 0, 0], p.LINK_FRAME)
    #         p.resetBaseVelocity(self.drone_id, linearVelocity=[velocity[0] * 0.5, velocity[1] * 0.5, velocity[2] * 0.5], angularVelocity=[0, 0, 0])

    #     # Update observation
    #     obs = self.current_state_space()
    #     observation = np.array(obs + list(self.prev_action))
        
    #     # Calculate distance to target after the action
    #     new_distance_to_target = obs[4]
        
    #     # Reward calculation
    #     reward = 0
        
    #     # Penalize staying in the same position
    #     if prev_distance_to_target == new_distance_to_target:
    #         reward -= 500  # Adjust the penalty as needed

    #     # Reward getting closer to the target
    #     if new_distance_to_target < prev_distance_to_target:
    #         reward += 20  # Increase this reward if you want the drone to prioritize moving toward the target
        
    #     # Additional penalty for colliding with objects
    #     collision_points = p.getContactPoints(bodyA=self.drone_id)
    #     if collision_points:
    #         reward -= 5  # Add a penalty for collision

    #     # Check if the drone is near the target (within 0.5 meters) to end the episode
    #     if abs(new_distance_to_target) < 0.5:
    #         self.done = True
    #         reward += 10  # Reward for reaching the target
    #     else:
    #         self.done = False

    #     # Penalize for going out of bounds
    #     if obs[3] > 2.5 or obs[3] < 0:
    #         self.done = True
    #         reward -= 1
        
    #     # Update previous rewards and step the simulation
    #     self.prev_rewards = self.total_reward
    #     self.total_reward = reward
    #     p.stepSimulation()
        
    #     return observation, reward, self.done, False, {}

    
    
    
    # def step(self, action):
    #     print("in step")
    #     # Take action and simulate environment
    #     target_x = np.clip(self.target_x + action[0], -5, 5)
    #     target_y = np.clip(self.target_y + action[1], -5, 5)
        
    #     self.move_drone_to_target(self.drone_id, target_x, target_y)
        
    #     # Get updated drone position
    #     drone_pos = p.getBasePositionAndOrientation(self.drone_id)[0]
    #     distance = np.sqrt((target_x - drone_pos[0])**2 + (target_y - drone_pos[1])**2)
        
    #     # Define a reward
    #     reward = -distance
    #     if distance < 0.1:  # Define when to end the episode
    #         terminated = True  
    #     else:   
    #         terminated = False 
        
    #     # Construct the observation
    #     obs = np.array([drone_pos[0], drone_pos[1], self.target_x, self.target_y], dtype=np.float32)
        
    #     if terminated:
    #         # Optionally reset the target position for the next episode
    #         self.target_x = np.random.uniform(-5, 5)
    #         self.target_y = np.random.uniform(-5, 5)

    #     return obs, reward, terminated, False, {}  # Return obs, reward, done, truncated, info
    
    
    # def move_drone_to_target(self, drone_id, target_x, target_y, speed=0.1):
    #     current_position = p.getBasePositionAndOrientation(self.drone_id)[0]
    #     current_x, current_y = current_position[0], current_position[1]

    #     # Move to target coordinates smoothly
    #     while True:
    #         # Calculate the distance to the target
    #         distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

    #         # If we are close enough to the target, break
    #         if distance < 0.1:
    #             break

    #         # Calculate the new position
    #         current_x += speed * (target_x - current_x) / distance
    #         current_y += speed * (target_y - current_y) / distance

    #         # Update the drone's position
    #         p.resetBasePositionAndOrientation(drone_id, [current_x, current_y, self.Z_AXIS], [0, 0, 0, 1])
            
    #         # Step the simulation
    #         p.stepSimulation()
    #         time.sleep(1./240.)
            
    def render(self, mode="human"):
        if self.render_mode is None:
            raise NotImplementedError("Render mode not specified")
        if mode == 'human':
            time.sleep(1/60)


    def close(self):
        p.disconnect()
    
    
    
    

