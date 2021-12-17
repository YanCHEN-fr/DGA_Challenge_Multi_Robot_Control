#!/usr/bin/env python

from models import *
from utils.utils import *

import torch
from cv_bridge import CvBridge
import numpy as np
import cv2
import random
import rospy
from sensor_msgs.msg import Image
import os

class YoloModel():
    '''
    wrapper of darknet
    '''
    def __init__(self):
        current_path="src/yolo/src/"
        model_def=current_path+"config/yolov3.cfg"
        self.img_size=416
        self.weights_path=current_path+"weights/yolov3.weights"
        class_path=current_path+"data/coco.names"
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.conf_thres=0.8
        self.nms_thres=0.4
        self.yolo=Darknet(model_def,img_size=self.img_size).to(self.device)
        self.Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor
        self.load_weight()
        self.yolo.eval()
        self.classes = load_classes(class_path)

    def load_weight(self):
        if self.weights_path.endswith(".weights"):
            # Load darknet weights
            self.yolo.load_darknet_weights(self.weights_path)
        else:
            # Load checkpoint weights
            self.yolo.load_state_dict(torch.load(self.weights_path))

    def imagePreProcessing(self, img):
        # Extract image and shape
        img = np.copy(img)
        img = img.astype(float)
        height, width, channels = img.shape

        self.h = height
        self.w = width

        # Determine image to be used
        self.padded_image = np.zeros((max(self.h, self.w), max(self.h, self.w), channels)).astype(float)

        # Add padding
        if (self.w > self.h):
            self.padded_image[(self.w - self.h) // 2: self.h + (self.w - self.h) // 2, :, :] = img
        else:
            self.padded_image[:, (self.h - self.w) // 2: self.w + (self.h - self.w) // 2, :] = img

        # Resize and normalize
        input_img = cv2.resize(self.padded_image, (self.img_size,self.img_size),interpolation = cv2.INTER_AREA) / 255.

        # Channels-first
        input_img = np.transpose(input_img, (2, 0, 1))

        # As pytorch tensor
        input_img = torch.from_numpy(input_img).float()
        input_img = input_img[None]

        return input_img

    def inference(self,img):
        '''
        predict based on 1 image
        '''
        img_t=self.imagePreProcessing(img)
        img_t=img_t.to(self.device)
        # Get detections
        with torch.no_grad():
            detections = self.yolo(img_t)
            detections = non_max_suppression(detections, self.conf_thres, self.nms_thres)
            detections=detections[0]
            if detections is not None:
                detections = rescale_boxes(detections, self.img_size, img.shape[:2])
                for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
                    lt=(int(x1),int(y1))
                    br=(int(x2),int(y2))
                    img=cv2.rectangle(img,lt,br,(255,255,255),thickness=2)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    label=self.classes[int(cls_pred)]
                    img=cv2.putText(img,label,lt, font, 1,(0,0,0),2,cv2.LINE_AA)
        return img



class YoloDetectPublisher():
    '''
    publish object detection information
    '''
    def __init__(self):
        self._cv_bridge=CvBridge()
        self.yolo = YoloModel()
        # Define subscribers
        self.image_sub = rospy.Subscriber("/husky/front_cam/camera/image", Image, self.callback, queue_size=1000)

        # Define publishers
        self.pub_ = rospy.Publisher("/result", Image, queue_size=1)
        rospy.loginfo("Launched node for object detection")
        # Spin
        rospy.spin()

    def callback(self,image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
        result=self.yolo.inference(cv_image)
        res_msg = self._cv_bridge.cv2_to_imgmsg(result)
        self.pub_.publish(res_msg)


if __name__=="__main__":
    # img=cv2.imread("data/samples/dog.jpg")
    # yolo=YoloModel()
    # result=yolo.inference(img)
    # cv2.imshow("result",result)
    # cv2.waitKey(0)

    rospy.init_node('result publisher')
    p=YoloDetectPublisher()