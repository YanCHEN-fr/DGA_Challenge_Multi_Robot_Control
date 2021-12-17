#!/usr/bin/env python

import math
from numpy.core.numeric import NaN
import cv_bridge
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


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

def rotate_counterclock(x, y, angle):
    newx=x*math.cos(angle)-y*math.sin(angle)
    newy=x*math.sin(angle)+y*math.cos(angle)
    return newx,newy

class Detection:
    def __init__(self,drone_N):
        self.bridge = cv_bridge.CvBridge()
        # Define subscribers
        self.rgb_sub = rospy.Subscriber("camera/rgb/image_raw".format(drone_N), Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('camera/depth/image_raw'.format(drone_N), Image, self.depth_callback)
        self.odom_sub=rospy.Subscriber("ground_truth/state".format(drone_N), Odometry,self.odom_callback)
        # self.yolo_sub=rospy.Subscriber("/person/image/position",Point,self.yolo_callback)
        # Define publishers
        self.pos_pub= rospy.Publisher("/person/real/position", Point, queue_size=1)

        self.f=554.254691191187
        self.angle=NaN
        self.depth=None
        self.cent_x=None
        self.cent_y=None
        self.x=None
        self.y=None
        self.z=None

    def rgb_callback(self,image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
        human_mask_down=(0,102,0)
        human_mask_up=(3,105,3)
        mask = cv2.inRange(image, human_mask_down, human_mask_up)
        # result = cv2.bitwise_and(image, image, mask=mask)
        # cv2.imshow("detect_result", result)
        # cv2.waitKey(0)
        if sum(sum(mask))>1000:
            # print("detect people")
            mass_x, mass_y = np.where(mask >0)
            self.cent_y = int(np.average(mass_x))
            self.cent_x = int(np.average(mass_y))
            image_center_x=320.5
            
            distance_x=self.cent_x-image_center_x
            self.angle=math.atan2(distance_x,self.f)
            self.calculate_position()

    def depth_callback(self,depth_msg):
        image = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding="passthrough")
        image=np.array(image)
        if self.cent_x is not None and self.cent_y is not None:
            self.depth=image[self.cent_y-2:self.cent_y+2,self.cent_x-2:self.cent_x+2].min()

    def odom_callback(self,odom_msg):
        self.x=odom_msg.pose.pose.position.x
        self.y=odom_msg.pose.pose.position.y
        self.z=odom_msg.pose.pose.position.z
        ox=odom_msg.pose.pose.orientation.x
        oy=odom_msg.pose.pose.orientation.y
        oz=odom_msg.pose.pose.orientation.z
        ow=odom_msg.pose.pose.orientation.w

        _,_,self.yaw=euler_from_quaternion(ox,oy,oz,ow)
    
    def yolo_callback(self, yolo_point):
        if self.x is not None and self.y is not None and self.z is not None:
            scale=self.z*1000/159.99941228826285
            image_center_x=160.5
            image_center_y=120.5
            pixel_size=math.sqrt(13)/68.5
            diff_x=(yolo_point.x-image_center_x)*scale*pixel_size
            diff_y=(240-yolo_point.y-image_center_y)*scale*pixel_size
            
            realx, realy=rotate_counterclock(diff_x, diff_y, self.yaw)
            people_x=realx+self.x
            people_y=realy+self.y

            person_position=Point()
            person_position.x=people_x
            person_position.y=people_y
            self.pos_pub.publish(person_position)
            # print("drone{}:x={},y={},yaw={}".format(drone_N, people_x,people_y,self.yaw))
        
    def calculate_position(self):
        if self.angle is not NaN and self.depth is not None and self.x is not None and self.y is not None:
            people_angle=self.yaw-self.angle
            people_x=math.cos(people_angle)*self.depth+self.x
            people_y=math.sin(people_angle)*self.depth+self.y

            if not math.isnan(people_x) and not math.isnan(people_y):
                person_position=Point()
                person_position.x=people_x
                person_position.y=people_y
                self.pos_pub.publish(person_position)
                # print("people angle={}, depth={}, ({},{})".format(people_angle, self.depth, self.x,self.y))
                # print("drone{}:x={},y={},angle={},yaw={}".format(drone_N, people_x,people_y,self.angle,self.yaw))


if __name__=="__main__":
    rospy.init_node('detect_person',anonymous=True)
    drone_N=int(rospy.get_param("~drone_N"))
    
    d=Detection(drone_N)
    rospy.spin()

