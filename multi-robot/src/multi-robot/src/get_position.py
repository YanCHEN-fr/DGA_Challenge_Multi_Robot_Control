#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
import math
import numpy as np
from gazebo_msgs.srv import GetModelState
from multi_robot.msg import robot_position
from rospy.numpy_msg import numpy_msg


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

        self.name = ['drone1', 'drone2', 'drone3', 'drone4']
    
    # def show_drone_position(self, drone_id):
    #     try:
    #         model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #         position = []
    #         for block in self._blockListDict.itervalues():
    #             blockName = str(block._name)
    #             resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
    #             rotq = resp_coordinates.pose.orientation
    #             angle = euler_from_quaternion(rotq.x, rotq.y, rotq.z, rotq.w)
    #             p = [resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z, angle[-1]]
    #             position.append(p)
    #         return position
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Get Model State service call failed: {0}".format(e))
    #         raise e

    def show_drone_position(self):
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        position = []
        for block in self.name:
            #blockName = str(block._name)
            resp_coordinates = model_coordinates(block, '')
            rotq = resp_coordinates.pose.orientation
            angle = euler_from_quaternion(rotq.x, rotq.y, rotq.z, rotq.w)
            p = [resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z, angle[-1]]
            position.append(p)
        return position

    # def show_gazebo_models(self):
    #     try:
    #         model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #         for block in self._blockListDict.itervalues():
    #             blockName = str(block._name)
    #             resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
    #             print('\n')
    #             print('State success = ', resp_coordinates.success)
    #             print(blockName)
    #             print("Position x: " + str(resp_coordinates.pose.position.x))
    #             print("Position y: " + str(resp_coordinates.pose.position.y))
    #             print("Position z: " + str(resp_coordinates.pose.position.z))
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Get Model State service call failed: {0}".format(e))
    #         raise e


rospy.init_node('publish_robot_position', anonymous=False)
global publisher 
publisher1 = rospy.Publisher('~robot_position1', numpy_msg(robot_position), queue_size=1)
publisher2 = rospy.Publisher('~robot_position2', numpy_msg(robot_position), queue_size=1)
publisher3 = rospy.Publisher('~robot_position3', numpy_msg(robot_position), queue_size=1)
publisher4 = rospy.Publisher('~robot_position4', numpy_msg(robot_position), queue_size=1)
rate = rospy.Rate(10)
show = State()

while not rospy.is_shutdown():
    all_positopn = show.show_drone_position()
    all_positopn = np.array(all_positopn, dtype=np.float32)
    #print(all_positopn)
    publisher1.publish(all_positopn[0])
    publisher2.publish(all_positopn[1])
    publisher3.publish(all_positopn[2])
    publisher4.publish(all_positopn[3])
    rate.sleep()
