import CMDcontrol
import rospy
import tf
import time
import threading
import numpy as np
import math
from math import sqrt
from geometry_msgs.msg import PoseWithCovarianceStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from kickBallOnly import kick_ball
from startDoorOnly import start_door
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ColorObject:
    def __init__(self,lower,upper,cName='none'):
        self.coLowerColor=lower
        self.coUpperColor=upper
        self.coResult={'find':False,'name':cName}

    def detection(self,image):
        blurred=cv2.GaussianBlur(image,(5,5),0)
        hsvImg=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsvImg,self.coLowerColor,self.coUpperColor)
        mask=cv2.dilate(mask,None,iterations=2)
        mask=cv2.erode(mask,None,iterations=2)
        contours=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        self.coResult['find']=False
        self.coResult['boundingR']=[]
        # print(len(contours))
        if len(contours)>0:
            c=max(contours,key=cv2.contourArea)
            self.coResult['boundingR']=cv2.minEnclosingCircle(c)
            return self.coResult['boundingR']
        else:
            return self.coResult['boundingR']


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

def action(act_name):
    print(f'执行动作: {act_name}')
    time.sleep(1)
    CMDcontrol.action_append(act_name)

def init_action_thread():
    th2 = threading.Thread(target=thread_move_action)
    th2.setDaemon(True)
    th2.start()

def thread_move_action():
    CMDcontrol.CMD_transfer()

def turn_to_redball(x,y,x_criterion=285,y_criterion=250,x_threshold=20,y_threshold=20):
    is_turn_done=False
    x_error=x-x_criterion
    y_error=y-y_criterion
    print(f'x:{x}, y:{y}')
    print(f'x_error:{x_error}, y_error:{y_error}')
    
    if (x_error<-x_threshold):
        print("左移动 ")
        action("leftsee40")
        action("turn004L")
        action("see")
    elif (x_error>x_threshold):
        print("右移动  ")
        action("rightsee40")
        action("turn004R")
        action("see")
    else:
        print("turn to redball ok")
        print(f'x:{x}, y:{y}')   
        is_turn_done=True
    return is_turn_done
    

lowerRed=np.array([0 , 80 , 0])
upperRed=np.array([11 , 255 , 255])
result=[]

def main():
    try:
        #初始化ros节点
        rospy.init_node('remove_flammable') 
        print('Start action thread')
        init_action_thread()
        time.sleep(1)
        
        
        while not rospy.is_shutdown():
            time.sleep(0.1)
            image_reader = ImgConverter()
            while not rospy.is_shutdown():
                head_ret, HeadOrg_img = image_reader.head_image()
                if HeadOrg_img is not None:
                    
                    ball=ColorObject(lowerRed,upperRed)
                    result=ball.detection(HeadOrg_img)
                    if len(result)==0:
                        print("视野中没有红色小球")
                        print("左转寻找小球")
                        action('turn005L')
                        time.sleep(1)
                        continue

                    (x,y) = result[0]
                    radius = result[1]
                    center = (int(x),int(y))
                    radius = int(radius)
                    cv2.circle(HeadOrg_img,center,radius,(0,255,0),2)  
                    cv2.imshow("image", HeadOrg_img)
                    cv2.waitKey(1)
                    result=turn_to_redball(x,y)
                    #if result==False:
                    #    continue

                else:
                    print("image nome wait")
                time.sleep(1)

    except rospy.ROSInterruptException:
        pass


if __name__=="__main__":
    main()