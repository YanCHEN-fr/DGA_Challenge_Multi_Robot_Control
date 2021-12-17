#!/usr/bin/env python

import rospy
import numpy as np
import cv2, cv_bridge
from sensor_msgs.msg import Image

class drone_camera:
    def __init__(self, drone_N):
        assert int(drone_N) in {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
        rospy.init_node("drone{}_kinect_vision".format(drone_N), anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/drone{}/camera/depth/image_raw'.format(drone_N), Image, self.image_callback)
        self.rgb_sub = rospy.Subscriber('/drone{}/camera/rgb/image_raw'.format(drone_N), Image, self.rgb_callback)


    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
        print(image[240,320])
        # cv2.namedWindow("drone{}_camera_window".format(drone_N), 1)
        # cv2.imshow("drone{}_camera_window".format(drone_N), image)
        # cv2.waitKey(1)

    def rgb_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,"rgb8")
        cv2.namedWindow("drone{}_rgb_camera_window".format(drone_N), 1)
        cv2.imshow("drone{}_rgb_camera_window".format(drone_N), image)
        if cv2.waitKey(1) & 0xff == ord('s'):
            cv2.imwrite("drone_image.jpg",image)
            print("write image ok")


if __name__ == '__main__':
    print("Let's view your drone camera")
    drone_N = input("Select a drone camera to view. (Options 1, 2, 3, 4, 5, 6, 7, 8, 9, 10): ")
    follower = drone_camera(drone_N)
    rospy.spin()


