# import gymnasium as gym
# from stable_baselines3 import PPO
# from stable_baselines3.common.env_util import DummyVecEnv

# # Import your environment
# from final import DroneCity

# def main():
#     # Create and wrap the environment
#     env = DroneCity(render_mode="human")
#     env = DummyVecEnv([lambda: env])

#     # Create the PPO model
#     model = PPO("MlpPolicy", env, verbose=1)

#     # Train the model
#     model.learn(total_timesteps=1000000)

#     # Save the model
#     model.save("ppo_drone_city2")

#     # Load the model (optional, in case you want to load and test later)
#     # model = PPO.load("ppo_drone_city")

#     # Evaluate the trained model
#     obs = env.reset()
#     for _ in range(100):  # Number of evaluation steps
#         action, _states = model.predict(obs)
#         obs, rewards, dones, info = env.step(action)
#         env.render()  # Render the environment (if implemented)

#     env.close()

# if __name__ == "__main__":
#     main()





import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
import os

# Import your environment
from final import DroneCity

def main():
    # Create and wrap the environment
    env = DroneCity(render_mode="human")
    env = DummyVecEnv([lambda: env])

    # Create a directory for saving checkpoints
    checkpoint_dir = "./checkpoints7/"
    os.makedirs(checkpoint_dir, exist_ok=True)

    # Create the PPO model
    model = PPO("MlpPolicy", env, verbose=1)

    # Define a checkpoint callback to save the model every n steps
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,  # Save the model every 10,000 steps
        save_path=checkpoint_dir,  # Directory to save checkpoints
        name_prefix="ppo_drone_city_checkpoint"
    )

    # Train the model with the checkpoint callback
    model.learn(total_timesteps=1000000, callback=checkpoint_callback)

    # Save the final model
    model.save("ppo_drone_city")

    # Load the model (optional, in case you want to load and test later)
    # model = PPO.load("ppo_drone_city")

    # Evaluate the trained model
    obs = env.reset()
    for _ in range(100):  # Number of evaluation steps
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()  # Render the environment (if implemented)

    env.close()

if __name__ == "__main__":
    main()
