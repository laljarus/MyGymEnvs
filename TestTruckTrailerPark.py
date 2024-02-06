# coding: utf-8

# In[1]:


import gym
#from matplotlib import pyplot as plt
import gym_examples
import pygame,sys
from pygame.locals import *
pygame.init()

FPS = 12
FramePerSec = pygame.time.Clock()

#env = gym.make('gym_examples/GridWorld-v0')
env = gym.make('gym_examples/TruckTrailerPark-v0')
env.reset()
done = False

speed = 0
steer_ag = 0

while True:
    

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()        
    
    pressed_keys = pygame.key.get_pressed()

    if pressed_keys[K_LEFT]:                           

        if steer_ag > -15:
            steer_ag = steer_ag - 0.25        

    if pressed_keys[K_RIGHT]:          
    
      if steer_ag < 15.0 :
            steer_ag = steer_ag + 0.25         

    if pressed_keys[K_UP]:
        if speed < 10:
            speed = speed + 0.25

    if pressed_keys[K_DOWN]:
        if speed > -10:
            speed = speed - 0.25

    if pressed_keys[K_s]:
        speed = 0
        steer_ag = 0

    #action = env.action_type.actions_indexes["IDLE"]
    #action = env.action_space.sample()
    #print("\nSpeed:\t",speed, "\t steer_ag:\t", steer_ag)
    action = {"speed":speed,"steering":steer_ag}
    obs, reward, done, info = env.step(action)
    env.render()

    FramePerSec.tick(FPS)
