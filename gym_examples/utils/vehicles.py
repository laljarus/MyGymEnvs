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
    def __init__(self, l_truck,l_trailer,l_hitch,x_truck_0,y_truck_0,yaw_truck_0,theta_0 = np.pi):

        self.truck = BicycleModelKinematics(l_truck/2,l_trailer/2,x_truck_0,y_truck_0,0,0,yaw_truck_0,0)

        self.x_truck = self.truck.x
        self.y_truck = self.truck.y
        self.yaw_truck = self.truck.yaw
        self.last_yaw_truck  = self.yaw_truck

        self.yaw_trailer = yaw_truck_0 + theta_0
        self.last_yaw_trailer = self.yaw_trailer

        self.R_truck = np.array([[np.cos(self.yaw_truck),-np.sin(self.yaw_truck),0],
                                 [np.sin(self.yaw_truck),np.cos(self.yaw_truck),0],
                                 [0,0,1]])

        self.R_trailer = np.array([[np.cos(self.yaw_trailer),-np.sin(self.yaw_trailer),0],
                                 [np.sin(self.yaw_trailer),np.cos(self.yaw_trailer),0],
                                 [0,0,1]])     
        self.l_hitch = l_hitch
        self.l_trailer = l_trailer
        self.x_trailer,self.y_trailer =  0
        self.tf_r_truck_to_trailer()

        self.trailer = BicycleModelKinematics(l_trailer/2,l_trailer/2,self.x_trailer,self.y_trailer,0,0,self.yaw_trailer,0)
        

    def tf_r_truck_to_trailer(self):

        self.R_truck = np.array([[np.cos(self.yaw_truck),-np.sin(self.yaw_truck),0],
                                 [np.sin(self.yaw_truck),np.cos(self.yaw_truck),0],
                                 [0,0,1]])

        self.R_trailer = np.array([[np.cos(self.yaw_trailer),-np.sin(self.yaw_trailer),0],
                                 [np.sin(self.yaw_trailer),np.cos(self.yaw_trailer),0],
                                 [0,0,1]])

        r_truck_wor = np.array([self.x_truck,self.y_truck,0])
        r_hitch_body = np.array([-self.l_hitch,0,0])
        r_trailer_body = np.array([-self.l_trailer/2,0,0])

        r_hitch_wor = r_truck_wor + np.dot(self.R_truck,r_hitch_body)
        r_trailer_wor = r_hitch_wor + np.dot(self.R_trailer,r_trailer_wor)


        self.x_trailer = r_trailer_wor[0]
        self.y_trailer = r_trailer_wor[1]

        #return r_trailer_wor[0],r_trailer_wor[1],r_trailer_wor[2]

    def update(self,LinearVelocity,SteeringAngle,dt):

        self.truck.update(LinearVelocity,SteeringAngle,dt)

        steerAg_trailer = (self.last_yaw_truck - self.last_yaw_trailer)*180/np.pi
        steerAg_trailer = steerAg_trailer %360

        self.trailer.update(LinearVelocity,steerAg_trailer,dt)

        self.last_yaw_truck = self.yaw_truck
        self.last_yaw_trailer = self.yaw_trailer

        self.x_truck = self.truck.x
        self.y_truck = self.truck.y
        self.yaw_truck = self.truck.yaw

        #self.x_trailer = self.trailer.x
        #self.y_trailer = self.trailer.y
        self.tf_r_truck_to_trailer()
        self.yaw_trailer = self.trailer.yaw
    
    def reset(self,x_truck_0,y_truck_0,yaw_truck_0,theta_0 = np.pi):
        
        self.x_truck =x_truck_0
        self.y_truck = y_truck_0
        self.yaw_truck = yaw_truck_0
        self.last_yaw_truck  = self.yaw_truck

        self.yaw_trailer = yaw_truck_0 + theta_0
        self.last_yaw_trailer = self.yaw_trailer
        self.tf_r_truck_to_trailer()
        





        

        
