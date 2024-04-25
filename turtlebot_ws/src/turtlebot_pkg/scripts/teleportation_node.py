#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Int32
#from turtlesim.msg import Pose
#from geometry_msgs.msg import Twist

turtle_x = 0
turtle_y = 0

vr = 0
vl = 0


topicNameXPOS='linear_X'
topicNameYPOS='linear_Y'

#def pose_callback(msg):
 #   global turtle_x, turtle_y
  #  # Update turtle position
    #turtle_x = msg.linear.x 
   # #turtle_y = msg.linear.y
    #turtle_x = msg.x
    #turtle_y = msg.y
   
def callback_X_joystick(msg):
    global turtle_x
    turtle_x = msg.data

def callback_Y_joystick(msg):
    global turtle_y
    turtle_y = msg.data
    
    
#Calculate the instantaneouse linear and angular velocity from the joystick position
def joystick_to_velocity(turtle_x, turtle_y, max_velocity):
    x_max = 11
    y_max = 11
    magnitude = np.sqrt((turtle_x/x_max)**2 + (turtle_y/y_max)**2)  # Magnitude
    #magnitude = np.sqrt((turtle_x)**2) + ((turtle_y)**2)
    scaled_magnitude = magnitude * max_velocity
    angle = np.arctan2(turtle_y, turtle_x)  # Angle in radians
    V = scaled_magnitude if scaled_magnitude <= max_velocity else max_velocity
    omega = angle  # Angular velocity in radians

    return V , omega
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
        vr = (2*V + omega*self.wheel_base) / (2*self.wheel_radius)
        vl = (2*V - omega*self.wheel_base) / (2*self.wheel_radius)
        #update robot pose
        self.x += dt * (self.wheel_radius/2) * (vr+vl) * np.cos(self.theta)
        self.y += dt * (self.wheel_radius/2) * (vr+vl) * np.sin(self.theta)
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
    #msg= Twist()
    
    rospy.init_node('turtle_position_plotter', anonymous=True)
    
    #rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    rospy.Subscriber(topicNameXPOS, Int32, callback_X_joystick)
    rospy.Subscriber(topicNameYPOS, Int32, callback_Y_joystick)
    
    #rospy.Subscriber('/turtle1/cmd_vel', Twist, Twist_callback)
    
    #rospy.spin()
    # Wait for the first message to arrive
    #rospy.wait_for_message('/turtle1/pose', Pose)
    #pub = rospy.Publisher('wheel_speeds', Twist, queue_size=10)
    #simulation parameters
    wheel_radius = 0.05  #[m] radius of wheels
    wheel_base = 0.2     #[m] distance between wheels
    dt = 0.1             #[s] time step for simulation
    total_time = 10.0    #[s] total time for simulation
    max_velocity=0.5     #[m/s] maximum velocity of robot

    #initialize robot
    robot = DifferentialDriveRobot(wheel_radius,wheel_base)

    #run simulation
    rate = rospy.Rate(1 / dt)  # Adjust rate based on time step
    start_time = rospy.get_time()
    
    #robot.plot_trajectory()
    
    while not rospy.is_shutdown():
            V, omega = joystick_to_velocity(turtle_x, turtle_y, max_velocity)
            robot.move(V, omega, dt)
            # Publish wheel speeds
            #twist_msg = Twist()
            #twist_msg.linear.x = vr  # Publish vr
            #twist_msg.angular.z = vl  # Publish vl
            #pub.publish(twist_msg)
            # Plot results
            #robot.plot_trajectory()
            #plt.plot(robot.history['x'],robot.history['y'],'ro')
            #plt.quiver(x, self.y, np.cos(self.theta), np.sin(self.theta))
            #plt.draw()
            #plt.pause(0.1)
            #rate.sleep()
            #i=i+1
            plt.plot(V,omega, 'bo')
            #plt.plot(turtle_x, turtle_y, 'bo')
            #plt.plot(turtle_x, turtle_y, 'ro')
            #plt.xlabel('X')
            #plt.ylabel('Y')
            plt.title('Trajectory of differential drive robot gow el loop')
            plt.axis('equal')
            plt.grid(True)
            plt.draw()
            plt.pause(0.1)
    plt.show()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
