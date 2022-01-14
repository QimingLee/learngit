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
            self.coResult['boundingR']=cv2.boundingRect(c)
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
        
        # cv2_img_rot = np.rot90(cv2_img)
        # self.pub_chest_rotated.publish(self.bridge.cv2_to_imgmsg(cv2_img_rot, "bgr8"))

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

def turn_to_flammable(x,y,w,x_criterion=228.0,y_criterion=315.5,w_criterion=329,x_threshold=10,y_threshold=10,w_threshold=20):
    is_turn_done=False
    x_error=x-x_criterion
    y_error=y-y_criterion
    w_error=w-w_criterion
    print(f'x:{x}, y:{y}, w:{w}')
    print(f'x_error:{x_error}, y_error:{y_error}, w_error:{w_error}')
    if (w_error>w_threshold):
        print("2后退")
        action("Back2Run")
    elif (w_error<-w_threshold):
        print("1前进")
        action("Forwalk01")
    elif (x_error<-x_threshold):
        print("2左移动 ")
        action("Left02move")
    elif (x_error>x_threshold):
        print("2右移动  ")
        action("Right02move")
    else:
        print("turn to flammable ok")
        print(f'x:{x}, y:{y}, w:{w}')   
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
                    #print("image update ok")
                    #第一步 寻找易燃物并获取颜色信息和区域位置信息
                    flammable=ColorObject(lowerRed,upperRed)
                    result=flammable.detection(HeadOrg_img)
                    if len(result)==0 or result[3]<30:
                        print("No flammable found")
                        print("左转寻找易燃物")
                        action('turn004L')
                        time.sleep(1)
                        continue

                    #cv2.rectangle(HeadOrg_img,(result[0],result[1]),(result[0]+result[2],result[1]+result[3]),(0,255,0),5)
                    #print("center point: (",result[0]+result[2]/2,",",result[1]+result[3]/2,",",result[3],")")
                    #cv2.imshow("image", HeadOrg_img)
                    #cv2.waitKey(1)
                    result=turn_to_flammable(result[0]+result[2]/2,result[1]+result[3]/2,result[3])
                    if result==False:
                        continue
                    #第二步 执行拆除
                    print("执行拆除")
                    action('removeAndRestore')
                    print("mission completed")
                    exit()
                else:
                    print("image nome wait")
                time.sleep(1)

    except rospy.ROSInterruptException:
        pass


if __name__=="__main__":
    main()