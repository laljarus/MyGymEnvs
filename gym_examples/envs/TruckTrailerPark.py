import gym
from gym import spaces
from importlib_metadata import metadata
import pygame,sys
import numpy as np
#from TruckTrailerReversing.Utils import TruckTrailerKinematicsModel
from gym_examples.utils.vehicles import *
from pygame.locals import *

class TruckTrailer_park(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 12}
    def __init__(self) -> None:
        super().__init__()
        self.window_width = 1080.0
        self.window_height = 720.0
        self.surface = None
        self.clock = None
        # Setting up color objects
        self.BLUE  = (0, 0, 255)
        self.RED   = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GREY = (128,128,128)
        self.ScaleFac = 10


        l_truck = 4
        l_trailer = 12
        l_hitch = 1
        self._agent_location = np.array([[0],[0],[0]])
        self._goal_location = np.array([[-10],[5],[np.pi/2]])

        self.vehicle = TruckTrailerKinematicsModel(l_truck,l_trailer,l_hitch,0,0,0)
        self.old_time = pygame.time.get_ticks() 

        low_bound = np.array([[-15],[-15],[-2*np.pi]])
        high_bound = np.array([[15],[15],[2*np.pi]])

        # agent_location - location of truck x,y and articulation angle theta = psi_1 - psi_2

        # goal location - location of trailer x,y, psi_2

        self.observation_space = spaces.Dict({"agent":spaces.Box(low=low_bound,high=high_bound,shape=(3,1), dtype= np.float32),
                                            "goal":spaces.Box(low=low_bound,high=high_bound,shape=(3,1), dtype= np.float32) })

        self.action_space = spaces.Dict({"speed":spaces.Box(low=-5,high=-5,shape=(1,), dtype= np.float32),
                                        "steering":spaces.Box(low=-15,high=-15,shape=(1,), dtype= np.float32)})
        
        self._agent_location = np.array([[self.vehicle.x_truck],[self.vehicle.y_truck],[self.vehicle.yaw_truck - self.vehicle.yaw_trailer]])                           

        #print("Truck Location:", self._agent_location)
        

    def _get_obs(self):
        return {"agent": self._agent_location, "goal": self._goal_location}

    def _get_info(self):
        return 0 

    def reset(self,return_info = False):
        self._agent_location = np.array([[0],[0],[0]])
        self._goal_location = np.array([[-10],[5],[np.pi/2]])
        self.vehicle.reset(0,0,0)
        self.old_time = pygame.time.get_ticks() 
        observation = self._get_obs()
        info = self._get_info()

        return (observation, info) if return_info else observation

    def step(self,action):

        speed = action.get('speed')
        steering = action.get('steering')

        time = pygame.time.get_ticks() 
        dt = (time - self.old_time )/1000.0
        self.vehicle.update(speed,steering,dt)
        reward = 0
        self._agent_location = np.array([[self.vehicle.x_truck],[self.vehicle.y_truck],[self.vehicle.yaw_truck - self.vehicle.yaw_trailer]])     
        self.old_time = pygame.time.get_ticks()
        
        observation = self._get_obs()
        info = self._get_info() 
        done = 0

        return observation, reward, done, info


    def render(self,mode="human"):

        if self.surface is None:
            pygame.init()
            pygame.display.init()
            self.surface = pygame.display.set_mode((self.window_width, self.window_height))
        if self.clock is None:
            self.clock = pygame.time.Clock()

        #self.surface = pygame.display.set_mode((self.window_width,self.window_height))
        self.surface.fill(self.GREY)
        pygame.display.set_caption("Simulation")
        self.drawParkLines()

        self.center_truck = self.WorldToImage(self.vehicle.x_truck,self.vehicle.y_truck,self.ScaleFac)
        self.center_trailer = self.WorldToImage(self.vehicle.x_trailer,self.vehicle.y_trailer,self.ScaleFac)

        image = pygame.Surface((20,40))
        image.fill(self.BLUE)
        image.set_colorkey(self.GREY)  
        image = pygame.transform.rotate(image,np.rad2deg(-self.vehicle.yaw_truck) )        
        rect = image.get_rect()        
        rect.center = self.center_truck        
        self.surface.blit(image,rect) 

        image_trailer = pygame.Surface((20,100))
        image_trailer.fill(self.BLUE)        
        image_trailer.set_colorkey(self.GREY)  
        image_trailer = pygame.transform.rotate(image_trailer,np.rad2deg(-self.vehicle.yaw_trailer))        
        rect_trailer = image_trailer.get_rect()
        rect_trailer.center = self.center_trailer
        self.surface.blit(image_trailer,rect_trailer) 

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        pygame.display.update()    
        self.clock.tick(self.metadata["render_fps"])        

    def drawParkLines(self):

        Hrzntl_1_x1 = 0
        Hrzntl_1_x2 = self.window_width
        Hrzntl_1_y1 = self.window_height *1/3
        Hrzntl_1_y2 = self.window_height *1/3

        Hrzntl_2_x1 = 0
        Hrzntl_2_x2 = self.window_width
        Hrzntl_2_y1 = self.window_height *2/3
        Hrzntl_2_y2 = self.window_height *2/3

        Vert_y1 = 0
        Vert_y2 = self.window_height *1/3
        Vert_y3 = self.window_height *2/3
        Vert_y4 = self.window_height

        NumOfLanes = 15
        LaneWidth = self.window_width/NumOfLanes       

        pygame.draw.line(self.surface,self.WHITE, (Hrzntl_1_x1,Hrzntl_1_y1),(Hrzntl_1_x2,Hrzntl_1_y2) )
        pygame.draw.line(self.surface,self.WHITE, (Hrzntl_2_x1,Hrzntl_2_y1),(Hrzntl_2_x2,Hrzntl_2_y2) )

        for i in range(NumOfLanes):

            Vert_x = (i+1)*LaneWidth


            pygame.draw.line(self.surface,self.WHITE, (Vert_x,Vert_y1),(Vert_x,Vert_y2) )
            pygame.draw.line(self.surface,self.WHITE, (Vert_x,Vert_y3),(Vert_x,Vert_y4) )

    def WorldToImage(self,x,y,factor):

        angle = -3/2*np.pi
        RotMat = np.array([[np.cos(angle), np.sin(angle)],[-np.sin(angle),np.cos(angle)]])        
        World_center = np.array([[self.window_width/2],[self.window_height/2]])        
        world_coord = np.array([x,y])
        world_coord = world_coord.reshape(2,1)        
        Img_coord = np.dot(RotMat,world_coord)*factor + World_center
        Img_coord = Img_coord.reshape(1,2).tolist()       
        
        return Img_coord[0]

    def close(self):
        if self.surface is not None:
            pygame.display.quit()
            pygame.quit()

