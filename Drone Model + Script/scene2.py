import pybullet as p
import pybullet_data
import random
import time
# from drone_control import drone_control


def check_keyboard_and_control(drone_id):
    print("Checking keyboard")
    force_magnitude = 10.0
    lift_force = 100.0
    yaw_torque = 10.0

    keys = p.getKeyboardEvents()
    force_x, force_y, force_z = 0, 0, 0
    torque_yaw = 0
    print(keys)
    # if ord('w') in keys: force_y = force_magnitude
    # if ord('s') in keys: force_y = -force_magnitude
    # if ord('a') in keys: force_x = -force_magnitude
    # if ord('d') in keys: force_x = force_magnitude
    # if ord('q') in keys: torque_yaw = yaw_torque
    # if ord('e') in keys: torque_yaw = -yaw_torque
    # if ord('i') in keys: force_z = lift_force
    # if ord('k') in keys: force_z = -lift_force
    if 65297 in keys:
        force_z = lift_force
    if 65298 in keys:
        force_z = -lift_force
    if 65295 in keys:
        force_x = -force_magnitude
    if 65296 in keys:
        force_x = force_magnitude
    if ord('q') in keys:
        torque_yaw = yaw_torque
    if ord('e') in keys:
        torque_yaw = -yaw_torque
    if ord('a') in keys:
        force_y = force_magnitude
    if ord('d') in keys:
        force_y = -force_magnitude 
    if keys is not None:
        # print(f"Force: {force_x, force_y, force_z}, Torque: {torque_yaw}")
        pass
    p.applyExternalForce(drone_id, -1, [force_x, force_y, force_z], [0, 0, 0], p.WORLD_FRAME)
    p.applyExternalTorque(drone_id, -1, [0, 0, torque_yaw], p.WORLD_FRAME)

# Connect to PyBullet
physicsClient = p.connect(p.GUI)

# Set up search path for built-in URDF files
p.setAdditionalSearchPath('/Users/aldrinvrodrigues/Engineering/SEM-5/Kodikon-4.0')
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -9.81)

# Define asset dimensions (assuming uniform size for blocks and cubes)
BLOCK_DIMENSIONS = [1, 1, 1]  # width, length, height for block
CUBE_DIMENSIONS = [1, 1, 1]    # width, length, height for cube

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
    # "block1": ([0, 0, 0], [0, 0, 0]),
    # "block2": ([2, -2, 0], [0, 0, 0]),
    # "block3": ([-2, 3, 0], [0, 0, 0]),
    # "block4": ([1, 4, 0], [0, 0, 0]),
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
    # # Find a valid position for the block
    # while True:
    #     random_position = [random.uniform(-5, 5), random.uniform(-5, 5), 0]
    #     if not is_overlapping(random_position, existing_positions, BLOCK_DIMENSIONS):
    #         assets[f"block{i}"] = (random_position, [0, 0, 0])
    #         existing_positions.append(random_position)
    #         break

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
                drone_id = p.loadURDF("cf2x.urdf", [0, 0, 0.1], baseOrientation=orientation)
                # check_keyboard_and_control(drone_id)
        else:
            p.loadURDF(f"{asset_name}.urdf", basePosition=position, baseOrientation=orientation)
    except:
        print(f"Failed to load asset: {asset_name}")
        obj_ids = p.loadSDF(f"{asset_name}.sdf")
        
        # If SDF loaded successfully, set positions for each object
        for obj_id in obj_ids:
            p.resetBasePositionAndOrientation(obj_id, position, orientation)


# Simulation loop
for i in range(100000):
    if drone_id is not None:
        check_keyboard_and_control(drone_id)
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()
