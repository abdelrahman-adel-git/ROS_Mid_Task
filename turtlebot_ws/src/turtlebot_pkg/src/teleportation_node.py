#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Int32
#from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

turtle_x = 0
turtle_y = 0

vr = 0
vl = 0

xmag = 0
ymag = 0

right_enc = 0
left_enc = 0
IMU_reading = 0

VR = Int32()
VL = Int32()

topicNameVR='vr'
topicNameVL='vl'
topicNameTwist = 'turtle_cmd_vel'
topicNameLeftEncoder = 'encoder_left'
topicNameRightEncoder = 'encoder_right'
topicNameIMU = 'MPU'


def differential_drive_to_car_velocity(left_enc, right_enc, wheel_base, wheel_radius):
    # Convert wheel speeds from rad/s to m/s
    
    linear_velocity_left = left_enc * wheel_radius
    linear_velocity_right = right_enc * wheel_radius
    
    # Calculate car linear velocity
    linear_velocity = (linear_velocity_left + linear_velocity_right) / 2.0
    
    # Calculate car angular velocity
    angular_velocity = (linear_velocity_right - linear_velocity_left) / wheel_base
    
    #print("velocity_feedback%s" %linear_velocity) 
    
    return linear_velocity, angular_velocity

def callback_encoder_right(message1):
    global right_enc
    right_enc = message1.data
    #print("angular velocity: %s" %right_enc) 
def callback_encoder_left(message2):
    global left_enc
    left_enc = message2.data
    #print("angular velocity: %s" %message2.data) 
def callback_IMU(message3):
    global IMU_reading
    IMU_reading = message3.data 
    #print("angular velocity: %s" %message3.data)  
    
def twist_callback(msg):
    global turtle_x, turtle_y
    # Update turtle positiona
    turtle_x = msg.linear.x
    turtle_y = msg.angular.z
    #print("Angular: %s" %msg.linear.x)
    #print("V: %s" %msg.angular.z)
            
    
    
    
#Calculate the instantaneouse linear and angular velocity from the joystick position
    
def get_wheel_velocities(V, omega, wheel_base, wheel_radius):
    vr = (2*V - omega*wheel_base) / (2*wheel_radius)
    vl = (2*V + omega*wheel_base) / (2*wheel_radius)
    return vr, vl
    
class DifferentialDriveRobot:
    def __init__(self, wheel_radius, wheel_base):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.x = 0.0 #initial x position
        self.y = 0.0 #initial y position
        self. theta = 0.0 #initial orientaion
        self.history = {'x': [], 'y': [],  'theta': []}

    def move(self,V,omega,dt):
        #Compute wheel speeds
        vrr = (2*V + omega*self.wheel_base) / (2*self.wheel_radius)
        vll = (2*V - omega*self.wheel_base) / (2*self.wheel_radius)

        #update robot pose
        self.x += dt * (self.wheel_radius/2) * (vrr+vll) * np.cos(self.theta)
        self.y += dt * (self.wheel_radius/2) * (vrr+vll) * np.sin(self.theta)
        self.theta += dt *(self.wheel_radius/self.wheel_base) * (vr-vl)
        #store history of positions and orientation
        self.history['x'].append(self.x)
        self.history['y'].append(self.y)
        self.history['theta'].append(self.theta)

    def plot_trajectory(self):
        #Plots the trajectory of the robot
        plt.plot(self.x,self.y,'ro')
        plt.quiver(self.x, self.y, np.cos(self.theta), np.sin(self.theta))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Trajectory of differential drive robot')
        plt.axis('equal')
        plt.grid(True)
def main():
    rospy.init_node('turtle_position_plotter', anonymous=True)
    
    rospy.Subscriber(topicNameLeftEncoder, Int32, callback_encoder_left)
    rospy.Subscriber(topicNameRightEncoder, Int32, callback_encoder_right)
    rospy.Subscriber(topicNameIMU, Int32, callback_IMU)
    
    rospy.Subscriber('/cmd_velocity', Twist, twist_callback)
    
    publeftmot=rospy.Publisher(topicNameVL, Int32, queue_size=5)
    pubrightmot=rospy.Publisher(topicNameVR, Int32, queue_size=5)
    
    gazebopub=rospy.Publisher("cmd_vel", Twist, queue_size=5)
    
    #rospy.spin()
    
    #simulation parameters
    wheel_radius = 0.05  #[m] radius of wheels
    wheel_base = 0.2     #[m] distance between wheels
    dt = 0.1             #[s] time step for simulation
    total_time = 10.0    #[s] total time for simulation
    max_velocity=0.5     #[m/s] maximum velocity of robot

    #initialize robot
    robot = DifferentialDriveRobot(wheel_radius,wheel_base)

    #run simulation
    ratePublisher = rospy.Rate(1/dt)  # Adjust rate based on time step
    #start_time = rospy.get_time()
    
    #robot.plot_trajectory() 
    
    while not rospy.is_shutdown():
            msg = Twist()
            message1 = Int32()
            message2 = Int32()
            message3 = Int32()
            V = turtle_x/100
            omega = -turtle_y*np.pi/180
            #print("angular velocity: %s" %right_enc)
            linear_velocity, angular_velocity = differential_drive_to_car_velocity(left_enc, right_enc, wheel_base, wheel_radius)
            #print("velocity_feedback%s" %linear_velocity) 
            robot.move(V, omega, dt)
            vr, vl = get_wheel_velocities(V, omega, robot.wheel_base, robot.wheel_radius)
            
            #print("V: %s" %V)
            #print("angle: %s" %omega)
            
            VR.data = int(vr)
            VL.data = int(vl)        
            
            # Publish wheel speeds	
            publeftmot.publish(VL)
            pubrightmot.publish(VR)	
         
            gazebo_msg = Twist()
  
            gazebo_msg.linear.x = -linear_velocity    
            gazebo_msg.angular.z = -IMU_reading*np.pi/180
            
            if (abs(linear_velocity) < 0.0):
                gazebo_msg.linear.x = 0
                
            if ((gazebo_msg.angular.z < 0.3) and (gazebo_msg.angular.z >-0.3)):
                gazebo_msg.angular.z = 0
            else :
                gazebo_msg.angular.z = -2.5*IMU_reading*np.pi/180
                
		
            gazebopub.publish(gazebo_msg)
            #print("linear velocity: %s" %gazebo_msg.linear.x)
            print("angular velocity: %s" %gazebo_msg.angular.z)

            ratePublisher.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
