#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
import math
import numpy as np
from gazebo_msgs.srv import GetModelState

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class State:

    def __init__(self):
        
        self._blockListDict = {
            'block_a': Block('drone1', ''),
            'block_b': Block('drone2', ''),
            'block_c': Block('drone3', ''),
            'block_d': Block('drone4', ''),
        }
    
    def show_drone_position(self, drone_id):
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            blockName = "drone"+str(drone_id)
            resp_coordinates = model_coordinates(blockName, '')
            rotq = resp_coordinates.pose.orientation
            angle = euler_from_quaternion(rotq.x, rotq.y, rotq.z, rotq.w)
            return resp_coordinates.pose.position.x,resp_coordinates.pose.position.y,resp_coordinates.pose.position.z, angle[-1]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def update_velocity(v_x, v_y, v_z, av_x, av_y, av_z):
    vel_msg3.linear.x = v_x
    vel_msg3.linear.y = v_y
    vel_msg3.linear.z = v_z
    vel_msg3.angular.x = av_x
    vel_msg3.angular.y = av_y
    vel_msg3.angular.z = av_z
    velocity_publisher3.publish(vel_msg3)

def take_off(drone_N):
    # Starts a new node
    assert int(drone_N) in {1,2,3,4}

    rospy.init_node('move_drone{}'.format(drone_N), anonymous=True)
    # check if motors are on
    if motor_on():
        print("control {}".format(drone_N))
        global velocity_publisher3
        velocity_publisher3 = rospy.Publisher('drone{}/cmd_vel'.format(drone_N), Twist, queue_size=10)

        global vel_msg3 
        vel_msg3 = Twist()
        show3 = State()
        # Receiveing the user's input
        krho=0.5
        kalpha=0.7

        take_off_hight = 1
        update_velocity(0, 0, 0, 0, 0, 0)

        while not rospy.is_shutdown():

            # Setting the current time for distance calculus
            #t0 = rospy.Time.now().to_sec()
            position = show3.show_drone_position(drone_N)
            position = np.array(position)

            # Drone takes off vertically at original point
            while abs(take_off_hight - position[2])>0.2 and not rospy.is_shutdown():

                # Takes actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                current_distance_z = position[2]
                v_z = krho*(take_off_hight - current_distance_z)
                update_velocity(0, 0, v_z, 0, 0, 0)
                position=show3.show_drone_position(drone_N)
                position = np.array(position)

            update_velocity(0, 0, 0, 0, 0, 0.2)           
    else:
        return 0


# Rosservice function to turn on the drone motors
def motor_on():
    rospy.wait_for_service('drone{}/enable_motors'.format(drone_N))
    try:
        motor_on = rospy.ServiceProxy('drone{}/enable_motors'.format(drone_N), EnableMotors, True)
        turn_on = motor_on(True)
        return turn_on
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':

    nodes = [[-16,-36,1],
        [7,-36,1],
        [7,-40,1],
        [28,-40,1],
        [28,-22,1],
        [0,-22,1],
        [-15,-24,1],
        [-30,-24,1]]
    print("Let's move your drone")
    drone_N = input("Select a drone to move. (Options 1, 2, 3, 4): ")
    # Testing our function
    # drone_N=1
    
    take_off(drone_N)