import numpy as np

class BicycleModelKinematics():
    
    def __init__(self,l_f,l_r,x0,y0,xdot0,ydot0,yaw0,yawRate0):
        
        self.l_f = l_f
        self.l_r = l_r
        self.x = x0
        self.y = y0
        self.xdot = xdot0
        self.ydot = ydot0
        self.yaw = yaw0
        self.yawRate = yawRate0
        self.sideSlip = 0
        self.steerAngle = 0
        
    def update(self,LinearVelocity,SteeringAngle,dt):
        
        # steering angle in deg
        # LinearVelocity in km/hr
        # dt  sample time in seconds
        
        SteeringAngle = self.Deg2Rad(SteeringAngle)
        self.sideSlip =  np.arctan(self.l_r/(self.l_f+self.l_r)*np.tan(SteeringAngle))
        velocity = LinearVelocity/3.6 # converting to m/s
        
        self.yawRate = velocity*np.cos(self.sideSlip)*(np.tan(SteeringAngle))/(self.l_f + self.l_r)
        
        self.yaw = (self.yaw + self.yawRate*dt)%(2*np.pi)
        
        self.xdot = velocity*np.cos(self.yaw + self.sideSlip)
        #self.xdot = velocity*np.cos(self.yaw )
        self.ydot =  velocity*np.sin(self.yaw + self.sideSlip)
        #self.ydot =  velocity*np.sin(self.yaw)
        
        self.x = self.x + self.xdot*dt
        self.y = self.y + self.ydot*dt
    
    def reset(self,x0,y0,xdot0,ydot0,yaw0,yawRate0):
        self.x = x0
        self.y = y0
        self.xdot = xdot0
        self.ydot = ydot0
        self.yaw = yaw0
        self.yawRate = yawRate0
    
    def Rad2Deg(self,ang):
        return ang*180/np.pi
    
    def Deg2Rad(self,ang):
        return ang*np.pi/180

class TruckTrailerKinematicsModel():
    def __init__(self,l_truck,l_rear_cog_truck,l_rear_hitch_truck,l_trailer,l_rear_cog_trailer,x_truck_0 = 0 ,y_truck_0 = 0, yaw_truck_0 = 0,theta_0 = 0):
        '''
        l_truck: length of truck
        l_rear_cog_truck: distance from rear wheel to center of gravity of truck
        l_rear_hitch_truck: distance from rear wheel to hitch of truck
        l_trailer: length of trailer
        l_rear_cog_trailer: distance from rear wheel to center of gravity of trailer
        x_truck_0: initial x position of truck
        y_truck_0: initial y position of truck
        yaw_truck_0: initial yaw of truck
        theta_0: initial angle between truck and trailer - articulation angle
        '''

        self.l_truck = l_truck
        self.l_f_truck = l_truck -  l_rear_cog_truck
        self.l_r_truck = l_rear_cog_truck
        self.l_hitch_truck = l_rear_cog_truck - l_rear_hitch_truck # distance from the cog of truck to the hitch
        self.l_trailer = l_trailer
        self.l_f_trailer = l_trailer - l_rear_cog_trailer
        self.l_r_trailer = l_rear_cog_trailer

        # truck position vector [x,,y,z]
        self.position_truck = np.array([x_truck_0,y_truck_0,0]) 

        # truck orientation vector [yaw, pitch, roll]
        self.orientation_truck = np.array([0,0,yaw_truck_0])    

        # position of hitch of truck in truck frame
        self.position_hitch_truck = np.array([0,-self.l_hitch_truck,0])

        # Rotation matrix from truck frame to world frame
        self.R_truck = np.array([[np.cos(self.orientation_truck[2]),-np.sin(self.orientation_truck[2]),0],
                                 [np.sin(self.orientation_truck[2]),np.cos(self.orientation_truck[2]),0],
                                 [0,0,1]],dtype=np.float32)
        
        # position of hitch of truck in world frame
        self.position_hitch_truck_world = self.position_truck + np.dot(self.R_truck,self.position_hitch_truck)

        # Position of the trailer in hitch frame
        self.position_trailer_hitch = np.array([0,-self.l_f_trailer,0])       

        # orientation of the trailer in world frame

        yaw_trailer = self.orientation_truck[2] - theta_0
        self.orientation_trailer = np.array([0,0,yaw_trailer])

        
        # Rotation matrix from hitch frame to world frame
        self.R_trailer = np.array([[np.cos(self.orientation_trailer[2]),-np.sin(self.orientation_trailer[2]),0],
                                 [np.sin(self.orientation_trailer[2]),np.cos(self.orientation_trailer[2]),0],
                                 [0,0,1]],dtype=np.float32)
        
        # position of trailer in world frame
        self.position_trailer_world = self.position_hitch_truck_world + np.dot(self.R_trailer,self.position_trailer_hitch)

        # velocity of truck
        self.velocity_truck = np.array([0,0,0])

        # angular velocity of truck
        self.angular_velocity_truck = np.array([0,0,0])

        # velocity of trailer
        self.velocity_trailer = np.array([0,0,0])

        # angular velocity of trailer
        self.angular_velocity_trailer = np.array([0,0,0])

    def update(self,LinearVelocity,SteeringAngle,dt):

        # position and orientation update of truck and trailer

        self.position_truck = self.position_truck + self.velocity_truck*dt
        self.orientation_truck = self.orientation_truck + self.angular_velocity_truck*dt
        self.orientation_trailer = self.orientation_trailer + self.angular_velocity_trailer*dt

        self.R_truck = np.array([[np.cos(self.orientation_truck[2]),-np.sin(self.orientation_truck[2]),0],
                                 [np.sin(self.orientation_truck[2]),np.cos(self.orientation_truck[2]),0],
                                 [0,0,1]],dtype=np.float32)
        
        self.position_hitch_truck_world = self.position_truck + np.dot(self.R_truck,self.position_hitch_truck)


        self.R_trailer = np.array([[np.cos(self.orientation_trailer[2]),-np.sin(self.orientation_trailer[2]),0],
                                    [np.sin(self.orientation_trailer[2]),np.cos(self.orientation_trailer[2]),0],
                                    [0,0,1]],dtype=np.float32)

        self.position_trailer_world = self.position_hitch_truck_world + np.dot(self.R_trailer,self.position_trailer_hitch)

        # velocity and angular velocity update of truck and trailer

        SteeringAngle = self.Deg2Rad(SteeringAngle)
        sideSlip =  np.arctan(self.l_r/(self.l_f+self.l_r)*np.tan(SteeringAngle))
        velocity_x = LinearVelocity/3.6 # converting to m/s
        
        yawRate = velocity_x*np.cos(sideSlip)*(np.tan(SteeringAngle))/(self.l_f + self.l_r)

        self.angular_velocity_truck = np.array([0,0,yawRate])
        self.velocity_truck = np.dot(self.R_truck,np.array([velocity_x,0,0]))

        # linear velocity of hitch point of truck in world frame
        yaw_truck = self.orientation_truck[2]
        self.velocity_hitch_world = self.velocity_truck + np.array([yawRate*self.l_hitch_truck*np.sin(yaw_truck) , 
                                                                    -yawRate*self.l_hitch_truck*np.cos(yaw_truck),0])

        # angular velocity of trailer
        # inverse of the rotation matrix of trailer
        R_trailer_inv = np.linalg.inv(self.R_trailer)
        velocity_hitch_trailer = np.dor(R_trailer_inv, self.velocity_hitch_world)

        # assumtion that the trailer wheels are moving only in the x direction i.e. the trailer axis is the center of rotation of trailer
        # angular velocity of trailer is linear velocity of hitch lateral directoin divided by the length of the trailer
        yawRate_trailer = velocity_hitch_trailer(1)/self.l_trailer
        self.angular_velocity_trailer = np.array([0,0,yawRate_trailer])

        # angular velocity of trailer
        yaw_trailer = self.orientation_trailer[2]
        self.velocity_trailer = self.velocity_hitch_world + np.array([yawRate_trailer*self.l_f_trailer*np.sin(yaw_trailer),
                                                                      -yawRate_trailer*self.l_f_trailer*np.cos(yaw_trailer),0])   

    def reset(self,x0 = 0,y0 = 0,xdot0 = 0,ydot0 = 0,yaw0 = 0, yawRate0 = 0, theta0 = 0,thetaDot0 = 0):

         # truck position vector [x,,y,z]
        self.position_truck = np.array([x0,y0,0]) 

        # truck orientation vector [yaw, pitch, roll]
        self.orientation_truck = np.array([0,0,yaw0])    

        # position of hitch of truck in truck frame
        self.position_hitch_truck = np.array([0,-self.l_hitch_truck,0])

        # Rotation matrix from truck frame to world frame
        self.R_truck = np.array([[np.cos(self.orientation_truck[2]),-np.sin(self.orientation_truck[2]),0],
                                 [np.sin(self.orientation_truck[2]),np.cos(self.orientation_truck[2]),0],
                                 [0,0,1]],dtype=np.float32)
        
        # position of hitch of truck in world frame
        self.position_hitch_truck_world = self.position_truck + np.dot(self.R_truck,self.position_hitch_truck)

        # Position of the trailer in hitch frame
        self.position_trailer_hitch = np.array([0,-self.l_f_trailer,0])       

        # orientation of the trailer in world frame

        yaw_trailer = self.orientation_truck[2] - theta0
        self.orientation_trailer = np.array([0,0,yaw_trailer])

        
        # Rotation matrix from hitch frame to world frame
        self.R_trailer = np.array([[np.cos(self.orientation_trailer[2]),-np.sin(self.orientation_trailer[2]),0],
                                 [np.sin(self.orientation_trailer[2]),np.cos(self.orientation_trailer[2]),0],
                                 [0,0,1]],dtype=np.float32)
        
        # position of trailer in world frame
        self.position_trailer_world = self.position_hitch_truck_world + np.dot(self.R_trailer,self.position_trailer_hitch)

        # velocity of truck
        self.velocity_truck = np.array([xdot0,ydot0,0])

        # angular velocity of truck
        self.angular_velocity_truck = np.array([0,0,yawRate0])

        yawRate_trailer = thetaDot0

        # velocity of trailer
        self.velocity_trailer = np.array([0,0,0])

        # angular velocity of trailer
        self.angular_velocity_trailer = np.array([0,0,yawRate_trailer])

    def Rad2Deg(self,ang):
        return ang*180/np.pi
    
    def Deg2Rad(self,ang):
        return ang*np.pi/180