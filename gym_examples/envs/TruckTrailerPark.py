import gym
from gym import spaces
import pygame
import numpy as np
from TruckTrailerReversing.Utils import TruckTrailerKinematicsModel
import utils.vehicles

class TruckTrailer_park(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 12}
    def __init__(self) -> None:
        super().__init__()
        self.window_width = 1080
        self.window_height = 720
        l_truck = 4
        l_trailer = 12
        l_hitch = 1
        self.vehicle = TruckTrailerKinematicsModel(l_truck,l_trailer,l_hitch,0,0,0)
        self.old_time = pygame.pygame.time.get_ticks() 

        low_bound = np.array([[-5],[-5],[0],[0]])
        high_bound = np.array([[5],[5],[np.pi],[np.pi]])

        self.observation_space = spaces.Dict({"agent":spaces.Box(low=low_bound,high=high_bound,shape=(4,1),dtype=np.float32),
                                            "goal":spaces.Box(low=low_bound,high=high_bound,shape=(4,1),dtype=np.float32) })

        self.action_space = spaces.Dict({"speed":spaces.Box(low=-5,high=-5,shape=(1,),dtype=np.float32),
                                        "steering":spaces.Box(low=-15,high=-15,shape=(1,),dtype=np.float32)})
        

    def _get_obs(self):
        pass

    def _get_info(self):
        pass

    def reset(self):
        self.vehicle.reset(0,0,0)
        self.old_time = pygame.time.get_ticks() 

    def step(self,action):

        speed = action.get('speed')
        steering = action.get('steering')

        time = pygame.time.get_ticks() 
        dt = time - self.old_time



        pass

    def render(self):
        pass

