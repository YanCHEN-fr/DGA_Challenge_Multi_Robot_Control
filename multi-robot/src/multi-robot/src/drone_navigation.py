#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
import math
import numpy as np
from gazebo_msgs.srv import GetModelState
from dron_path import nodes
from nav_msgs.msg import Odometry


def angle_wrap(a):

    ## limit the angle between -pi and pi ##
     
    while(a>math.pi):
        a = a - 2*math.pi
    while(a <= -math.pi):
        a = a + 2*math.pi

    return a

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

class Move:
    def __init__(self, drone_id, path_nodes):
        self.odom_sub=rospy.Subscriber("ground_truth/state".format(drone_id), Odometry,self.odom_callback)
        self.velocity_pub = rospy.Publisher('cmd_vel'.format(drone_id), Twist, queue_size=10)
        self.position=np.array([])
        self.take_off(drone_id,path_nodes)

    def odom_callback(self, odom_msg):
        x=odom_msg.pose.pose.position.x
        y=odom_msg.pose.pose.position.y
        z=odom_msg.pose.pose.position.z
        ox=odom_msg.pose.pose.orientation.x
        oy=odom_msg.pose.pose.orientation.y
        oz=odom_msg.pose.pose.orientation.z
        ow=odom_msg.pose.pose.orientation.w

        _,_,self.yaw=euler_from_quaternion(ox,oy,oz,ow)
        self.position=np.array([x, y, z, self.yaw])

    def update_velocity(self, v_x, v_y, v_z, av_x, av_y, av_z):
        vel_msg3=Twist()
        vel_msg3.linear.x = v_x
        vel_msg3.linear.y = v_y
        vel_msg3.linear.z = v_z
        vel_msg3.angular.x = av_x
        vel_msg3.angular.y = av_y
        vel_msg3.angular.z = av_z
        self.velocity_pub.publish(vel_msg3)

    def take_off(self, drone_N, path_nodes):
        assert int(drone_N) in {1,2,3,4,5,6,7,8,9,10}
        # rospy.init_node('move_drone{}'.format(drone_N), anonymous=True)
        # check if motors are on

        if motor_on():
            # print("control {}".format(drone_N))
            # Receiveing the user's input
            krho=0.5
            kalpha=0.7

            take_off_hight = path_nodes[0][-1]
            self.update_velocity(0, 0, 0, 0, 0, 0)

            while not rospy.is_shutdown():
                # check if get odometry data
                if self.position.size ==0:
                    continue
                # print(self.position)
                # Drone takes off vertically at original point
                while abs(take_off_hight - self.position[2])>0.2 and not rospy.is_shutdown():
                    current_distance_z = self.position[2]
                    v_z = krho*(take_off_hight - current_distance_z)
                    self.update_velocity(0, 0, v_z, 0, 0, 0)

                print("drone{} take off ready".format(drone_N))

                # The drone fly along the trajectory
                for node in path_nodes:
                    # print("drone{} goes to target{}".format(drone_N,node))                
                    
                    error=(node - self.position[0:3])[0:2]
                    AngleToGoal = angle_wrap(math.atan2(error[1], error[0])-self.position[3])
                    while abs(AngleToGoal)>0.2 and not rospy.is_shutdown():
                        error=(node - self.position[0:3])[0:2]
                        AngleToGoal = angle_wrap(math.atan2(error[1], error[0])-self.position[3])
                        av_z = kalpha * AngleToGoal
                        self.update_velocity(0, 0, 0, 0, 0, av_z)
                       
                    # move the droen to the target point
                    while (np.linalg.norm((node - self.position[0:3])[0:2])>0.5) and not rospy.is_shutdown():
                        error=(node - self.position[0:3])[0:2]
                        goalDist = np.linalg.norm(error)
                        AngleToGoal = angle_wrap(math.atan2(error[1], error[0])-self.position[3])
                        v_x = krho*goalDist
                        av_z = kalpha * AngleToGoal
                        v_z=krho*(node[2]-self.position[2])
                        self.update_velocity(v_x, 0, v_z, 0, 0, av_z)
                                                
                    # print("drone{} arrives at target{}".format(drone_N,node))
            else:
                return 0


# Rosservice function to turn on the drone motors
def motor_on():
    rospy.wait_for_service('enable_motors')
    try:
        motor_on = rospy.ServiceProxy('enable_motors', EnableMotors, True)
        turn_on = motor_on(True)
        return turn_on
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':

    rospy.init_node('move_drone', anonymous=True)
    drone_N=int(rospy.get_param("~drone_N"))
    path_id=int(rospy.get_param("~path_id"))
    
    print("move drone {} with path id={}".format(drone_N, path_id))
    # drone_N = input("Select a drone . (Options 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0 for all drones): ")
    # Testing our function
    # drone_N=1
    path_nodes=nodes[path_id-1]
    m=Move(drone_N,path_nodes)