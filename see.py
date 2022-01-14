#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class ImgConverter():
    def __init__(self):
        self.bridge = CvBridge()        
        self.sub_head = rospy.Subscriber('/usb_cam_head/image_raw', Image, self.cb_head)
        # self.pub_chest_rotated = rospy.Publisher("/usb_cam/image_rotated", Image)
        self.img_head = None
        

    def cb_head(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_head = cv2_img
    def head_image(self):
        if self.img_head is not None:
            return  True, self.img_head
        return False, self.img_head



def main():
    try: 
        rospy.init_node('image_listener')
        print('Node init')
        image_reader = ImgConverter()
        
        while not rospy.is_shutdown():
            head_ret, HeadOrg_img = image_reader.head_image()
            if HeadOrg_img is not None:
                print("image update ok")
                cv2.imshow('image', HeadOrg_img)
                cv2.waitKey(1)
                
            else:
                print("image nome wait")
            time.sleep(1)
           
    
    except rospy.ROSInterruptException:
        pass
    
    
# testing
if __name__ == '__main__':
    main()

