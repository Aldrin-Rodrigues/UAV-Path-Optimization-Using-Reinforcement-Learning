import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the drone
drone_id = p.loadURDF("git-kodikon/UAV-Path-Optimization-Using-Reinforcement-Learning/cf2x.urdf", [0, 0, 1], useFixedBase=False)
mass = p.getDynamicsInfo(drone_id, -1)[0]
gravity = 9.81

p.setGravity(0,0,-gravity)
# Rotor indices
rotor_indices = [0, 1, 2, 3]  # Adjust according to your URDF

# Thrust needed per rotor to hover
hover_thrust = mass * gravity / len(rotor_indices)

# # PID controller class
# class PIDController:
#     def __init__(self, Kp, Ki, Kd):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.integral = 0
#         self.prev_error = 0

#     def compute(self, target, current):
#         error = target - current
#         self.integral += error
#         derivative = error - self.prev_error
#         output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
#         self.prev_error = error
#         return output

# # Controllers for altitude, pitch, and roll
# altitude_controller = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
# pitch_controller = PIDController(Kp=0.8, Ki=0.05, Kd=0.02)  # Pitch stabilization
# roll_controller = PIDController(Kp=1.2, Ki=0.05, Kd=0.02)   # Roll stabilization

# # Target settings
# target_altitude = 1.0
# target_pitch = -0.1  # Set forward tilt angle (negative to tilt forward)
# target_roll = 0      # Keep the drone level in roll

# # Variable to track altitude state
# altitude_reached = False


# p.setDamping(drone_id, linearDamping=3)

# # Simulation loop
# dampner = 0.1


thrust_z = hover_thrust
while True:
    # Get the drone's position and orientation
    pos, orientation = p.getBasePositionAndOrientation(drone_id)

    # # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-50, cameraTargetPosition=pos)

    # current_altitude = pos[2]

    # # Altitude adjustment
    # altitude_adjustment = altitude_controller.compute(target_altitude, current_altitude)

    # # Get current pitch and roll from orientation
    # _, pitch, roll = p.getEulerFromQuaternion(orientation)

    # if not altitude_reached:
    #     # Adjust thrust to reach the target altitude
    #     thrust_front_left = hover_thrust + altitude_adjustment
    #     thrust_front_right = hover_thrust + altitude_adjustment
    #     thrust_rear_left = hover_thrust + altitude_adjustment
    #     thrust_rear_right = hover_thrust + altitude_adjustment
        
    #     # Check if the drone has reached the target altitude
    #     if abs(current_altitude - target_altitude) < 0.05:  # Small tolerance
    #         altitude_reached = True  # Altitude reached
    # else:
    #     # Once at altitude, compute pitch adjustment to move forward
    #     pitch_adjustment = pitch_controller.compute(target_pitch, pitch)
    #     roll_adjustment = roll_controller.compute(target_roll, roll)

    #     # Maintain Z-thrust for altitude

    #     # Adjust thrusts based on pitch and roll for forward motion
    #     # Forward thrust in the X-direction
    #     thrust_front_left = thrust_z + pitch_adjustment
    #     thrust_front_right = thrust_z + pitch_adjustment
    #     thrust_rear_left = thrust_z
    #     thrust_rear_right = thrust_z

    #     # Use roll adjustment to keep the drone level
    #     thrust_rear_left -= roll_adjustment
    #     thrust_rear_right += roll_adjustment

    # Clamp thrust values to ensure they do not exceed limits
    # thrust_limit = 10  # Maximum thrust per rotor
    # thrust_front_left = max(0, min(thrust_front_left, thrust_limit))
    # thrust_front_right = max(0, min(thrust_front_right, thrust_limit))
    # thrust_rear_left = max(0, min(thrust_rear_left, thrust_limit))
    # thrust_rear_right = max(0, min(thrust_rear_right, thrust_limit))


    # Apply thrust values to rotors
    # rotor_speeds = [
    #     thrust_front_left,
    #     thrust_front_right,
    #     thrust_rear_left,
    #     thrust_rear_right
    # ]

    rotor_speeds = [
        1000,1000,1000,1000
    ]

    # for i, rotor_index in enumerate(rotor_indices):
    #     thrust = rotor_speeds[i]
    #     # Apply thrust to each rotor in appropriate direction
    #     if rotor_index in [0, 1]:  # Front rotors (X-direction thrust)
    #         p.applyExternalForce(drone_id, rotor_index, [thrust * 0.1, thrust * 0.1, thrust_z], [0, 0, 0], p.LINK_FRAME)
    #     else:  # Rear rotors
    #         p.applyExternalForce(drone_id, rotor_index, [thrust * 0.1, thrust * 0.1, thrust_z], [0, 0, 0], p.LINK_FRAME)
    # print(p.getBaseVelocity(drone_id))
    # Step the simulation
    keys = p.getKeyboardEvents()
    velocity = p.getBaseVelocity(drone_id)[0]
    ang_velocity = p.getBaseVelocity(drone_id)[1]
    if 65297 in keys:
        for i in rotor_indices:
            thrust = rotor_speeds[i]
            if i in [0,2]:
                p.applyExternalForce(drone_id, i, [thrust, 0, thrust_z], [0,0,0], p.LINK_FRAME)
            else:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z], [0,0,0], p.LINK_FRAME)
    # elif 65296 in keys:
    #     p.applyExternalTorque(drone_id, -1, [0, 0, 1], p.LINK_FRAME)
    elif 65296 in keys:
        for i in rotor_indices:
            if i in [0,1]:
                p.applyExternalForce(drone_id, i, [10, 0, thrust_z], [0,0,0], p.LINK_FRAME)
            else:
                p.applyExternalForce(drone_id, i, [-10, 0, thrust_z], [0,0,0], p.LINK_FRAME)
    elif 65295 in keys:
        for i in rotor_indices:
            if i in [0,1]:
                p.applyExternalForce(drone_id, i, [-10, 0, thrust_z], [0,0,0], p.LINK_FRAME)
            else:
                p.applyExternalForce(drone_id, i, [10, 0, thrust_z], [0,0,0], p.LINK_FRAME)
    elif ord('q') in keys:
        for i in rotor_indices:
            if i in [0,1]:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z+3], [0,0,0], p.LINK_FRAME)
            else:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z-3], [0,0,0], p.LINK_FRAME)
    elif ord('e') in keys:
        for i in rotor_indices:
            if i in [0,1]:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z-3], [0,0,0], p.LINK_FRAME)
            else:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z+3], [0,0,0], p.LINK_FRAME)
    elif ord('f') in keys:
        for i in rotor_indices:
            if i in [0,3]:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z+3], [0,0,0], p.LINK_FRAME)
            else:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z-3], [0,0,0], p.LINK_FRAME)
    elif ord('b') in keys:
        for i in rotor_indices:
            if i in [0,3]:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z-3], [0,0,0], p.LINK_FRAME)
            else:
                p.applyExternalForce(drone_id, i, [0, 0, thrust_z+3], [0,0,0], p.LINK_FRAME)
    elif ord('u') in keys:
        for i in rotor_indices:
            p.applyExternalForce(drone_id, i, [0, 0, thrust_z+5], [0,0,0], p.LINK_FRAME)
    elif ord('d') in keys:
        for i in rotor_indices:
            p.applyExternalForce(drone_id, i, [0, 0, 0], [0,0,0], p.LINK_FRAME)
    else:
        # pass
        for i in rotor_indices:
            p.applyExternalForce(drone_id, i, [0,0,thrust_z], [0,0,0], p.LINK_FRAME)
        p.resetBaseVelocity(drone_id, linearVelocity = [velocity[0]*0.5, velocity[1]*0.5, velocity[2]*0.5], angularVelocity = [0, 0, 0])
            # velocity = velocity*dampner

    print(p.getBaseVelocity(drone_id))
    p.stepSimulation()
    time.sleep(1/240)  # Simulate real-time at 240Hz
