from gym.envs.registration import register
from importlib_metadata import entry_points

register(
    id="gym_examples/GridWorld-v0",
    entry_point="gym_examples.envs:GridWorldEnv",
)

register(
    id= "gym_examples/MyGridWorld-v0",
    entry_point= "gym_examples.envs:MyGridWorldEnv",
)