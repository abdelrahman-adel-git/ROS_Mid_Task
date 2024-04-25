#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
import matplotlib.pyplot as plt

# Global variables to store turtle position
turtle_x = []
turtle_y = []

def pose_callback(msg):
    global turtle_x, turtle_y
    # Update turtle position
    turtle_x.append(msg.x)
    turtle_y.append(msg.y)

def plot_turtle_position():
    rospy.init_node('turtle_position_plotter', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # Wait for the first message to arrive
    rospy.wait_for_message('/turtle1/pose', Pose)

    # Plot turtle position in real-time
    plt.ion()  # Turn on interactive mode
    plt.figure()
    plt.title('Turtle Position')
    plt.xlabel('X')
    plt.ylabel('Y')

    while not rospy.is_shutdown():
        plt.plot(turtle_x, turtle_y, 'b-')
        plt.draw()
        plt.pause(0.1)

if __name__ == '__main__':
    try:
        plot_turtle_position()
    except rospy.ROSInterruptException:
        pass

 

 
