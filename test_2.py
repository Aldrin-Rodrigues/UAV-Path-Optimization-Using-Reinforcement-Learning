import numpy as np
from final import DroneCity

# Initialize the DroneCity environment
env = DroneCity()

# Number of episodes and steps per episode to test
num_episodes = 10000
steps_per_episode = 100
env.reset()

for episode in range(num_episodes):
    print(f"\n--- Starting Episode {episode + 1} ---")
    # observation = env.reset()
    # print(f"Initial Observation: {observation}")

    for step in range(steps_per_episode):
        # Random action selection for testing
        action = 0
        print(f"\nStep {step + 1}")
        print(f"Action Taken: {action}")

        # Take a step in the environment
        observation, reward, done, _, info = env.step(action= 0)

        # Output the results of the step
        print(f"Observation: {observation}")
        print(f"Reward: {reward}")
        print(f"Done: {done}")

        # If the episode is done, break out of the loop
        if done:
            print(f"Episode finished after {step + 1} steps.")
            break

# Close the environment after the test
env.close()
print("\nTesting completed.")
