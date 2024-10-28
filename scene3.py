import pybullet as p
import pybullet_data
import random
import time
import numpy as np
# from drone_control import drone_control

# Connect to PyBullet
physicsClient = p.connect(p.GUI)

# Set up search path for built-in URDF files
p.setAdditionalSearchPath('/Users/aldrinvrodrigues/Engineering/SEM-5/Kodikon-4.0')
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, 0)

# Define asset dimensions (assuming uniform size for blocks and cubes)
BLOCK_DIMENSIONS = [1, 1, 1]  # width, length, height for block
CUBE_DIMENSIONS = [1, 1, 1]    # width, length, height for cube
    
    
    
def get_next_coordinates():
    # Placeholder for your RL algorithm to get next coordinates
    # Here, we're just returning random coordinates for demonstration
    
    return np.random.uniform(-5, 5), np.random.uniform(-5, 5)
    
    

def move_drone_to_target(drone_id, target_x, target_y, speed=0.1):
    current_position = p.getBasePositionAndOrientation(drone_id)[0]
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
        p.resetBasePositionAndOrientation(drone_id, [current_x, current_y, 0.1], [0, 0, 0, 1])
        
        # Step the simulation
        p.stepSimulation()
        time.sleep(1./240.)



# Function to check for overlap
def is_overlapping(new_position, existing_positions, dimensions):
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


# Define positions for city assets using unique names but referencing the same URDF
assets = {
    "plane": ([-5, -5, -0.1], [0, 0, 0]),
    "stadium": ([0, 0, 0], [0, 0, 0]),
    "cube1": ([3, 0, 1], [0, 0, 0]),
    "cube2": ([3, 1, 1], [0, 0, 0]),
    "cube3": ([-3, -2, 1], [0, 0, 0]),
    "cube4": ([4, -1, 1], [0, 0, 0]),
    "cube5": ([-1, -4, 1], [0, 0, 0]),
    "cf2x" : ([0, 0, 0], [0, 0, 0]),
}

# Store existing positions
existing_positions = [assets[asset][0] for asset in assets]

# Generate additional unique blocks and cubes at random positions
for i in range(6, 15):
    # Find a valid position for the cube
    while True:
        random_position = [random.uniform(-5, 5), random.uniform(-5, 5), 1]
        if not is_overlapping(random_position, existing_positions, CUBE_DIMENSIONS):
            assets[f"cube{i}"] = (random_position, [0, 0, 0])
            existing_positions.append(random_position)
            break

# Load the assets
for asset_name, (position, euler_orientation) in assets.items():
    orientation = p.getQuaternionFromEuler(euler_orientation)
    try:
        # if "block" in asset_name:
        #     p.loadURDF("block.urdf", basePosition=position, baseOrientation=orientation)
        if "cube" in asset_name:
            p.loadURDF("cube.urdf", basePosition=position, baseOrientation=orientation)
        elif "cf2x" in asset_name:
                drone_id = p.loadURDF("Drone Model + Script/cf2x.urdf", [0, 0, 0.1], baseOrientation=orientation)
                # check_keyboard_and_control(drone_id)
        else:
            p.loadURDF(f"{asset_name}.urdf", basePosition=position, baseOrientation=orientation)
    except:
        print(f"Failed to load asset: {asset_name}")
        obj_ids = p.loadSDF(f"{asset_name}.sdf")
        
        # If SDF loaded successfully, set positions for each object
        for obj_id in obj_ids:
            p.resetBasePositionAndOrientation(obj_id, position, orientation)
            
while True:
    target_x, target_y = get_next_coordinates()  # Get new target coordinates
    move_drone_to_target(drone_id, target_x, target_y)  # Move drone to target