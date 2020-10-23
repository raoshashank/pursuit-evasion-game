#!/usr/bin/env python
from __future__ import print_function

import roslib
import numpy as np
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import IPython
import tf
import cv_bridge
from imutils.object_detection import non_max_suppression
from imutils import paths
import image_geometry as img_geo

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/tb3_0/camera/rgb/image_opencv",Image,queue_size= 10)
    self.evader_position_pub = rospy.Publisher("/evader_position",PoseStamped,queue_size=10) 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/tb3_0/rrbot/camera1/image_raw",Image,self.callback)
    self.depth_sub = rospy.Subscriber("/kinect_camera_bot/depth/image_raw",Image,self.depth_callback)
    self.hog = cv2.HOGDescriptor()
    self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    self.camera_model = img_geo.StereoCameraModel()
    IPython.embed()
  def depth_callback(self,msg):
    #print(len(msg))
    depth_image = self.bridge.imgmsg_to_cv2(msg,"passthrough")
    #IPython.embed()

  def callback(self,data):
    try:
      converted_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    try:
      #cascade_src = '/home/raoshashank/catkin_ws/src/pursuit_evasion/src/human_detection.xml'
      #car_cascade = cv2.CascadeClassifier(cascade_src)
      gray = cv2.cvtColor(converted_img, cv2.COLOR_BGR2GRAY)
      (rects,weights) = self.hog.detectMultiScale(gray,winStride=(4,4),padding = (8,8),scale = 1.05)    
      #cars = car_cascade.detectMultiScale(gray, 1.1, 1)
      #IPython.embed()
      rects = np.array([[x,y,x+w,y+h] for (x,y,w,h) in rects])
      pick = non_max_suppression(rects,probs = None,overlapThresh = 0.65)
      for (xA,yA,xB,yB) in pick:
        cv2.rectangle(converted_img,(xA,yA),(xB,yB),(0,255,0),2)
      cv2.imshow("img",converted_img)
      evader_position = PoseStamped()
      cv2.waitKey(0)   
      cv2.destroyAllWindows()
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  rate = rospy.Rate(10.0)
  listener = tf.TransformListener()
  while not rospy.is_shutdown():
      try:
          now = rospy.Time.now()
          listener.waitForTransform("/tb3_0/camera_link","/map", now, rospy.Duration(4.0))
          (trans,rot) = listener.lookupTransform("/tb3_0/camera_link","/map", now)
      except Exception as e:
          print(e)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)