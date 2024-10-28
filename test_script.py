# Import the environment and Gym
from stable_baselines3.common.env_checker import check_env
from final import DroneCity

# Initialize the environment
env = DroneCity()

# Check if the environment follows the Gym API
# check_env(env, warn=True)

# Reset the environment to get the initial observation
obs = env.reset()
print("Initial Observation:", obs)

# Sample a random action from the action space
action = env.action_space.sample()
print("Sampled Action:", action)

# Take one step in the environment with the sampled action
obs, reward, terminated, truncated, info = env.step(action)
print("Observation after one step:", obs)
print("Reward:", reward)
print("Done:", terminated)
print("Info:", info)

# Close the environment
env.close()


# # Import the environment and Gym
# from stable_baselines3.common.env_checker import check_env
# from DroneCity import DroneCity

# # Initialize the environment
# env = DroneCity()

# # Check if the environment follows the Gym API
# print("Checking environment...")
# check_env(env, warn=True)

# # Reset the environment to get the initial observation
# obs = env.reset()
# print("Initial Observation:", obs)

