#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import json
import time
from geometry_msgs.msg import Twist
import numpy as np
from imutils.object_detection import non_max_suppression

class image_converter:

  def __init__(self):

    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("/tb3_0/rrbot/camera1/image_raw/image_opencv",Image,queue_size= 100)
    self.pub_vel = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/tb3_0/rrbot/camera1/image_raw",Image,self.callback)
    self.box_area = []
    self.flag = 0
    self.hog = cv2.HOGDescriptor()
    self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

  def callback(self,data):
    try:
      self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def search_human(self):
    human, weights = [], []
    switch = 0
    while len(human) == 0:
      gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
      human, weights = self.hog.detectMultiScale(gray, winStride=(4, 4), padding=(8, 8), scale=1.05)
      print("HUMAN")
      print(human)
      if len(human) != 0:
        break
      else:
        if switch == 0:
          switch = 1
          self.pan_cam()
        else:
          switch = 0
          self.move_f()
    return human, weights

  def move_f(self):
      vel_msg = Twist()
      vel_msg.linear.x = 0.5
      self.pub_vel.publish(vel_msg)
      time.sleep(1)
      vel_msg.linear.x = 0
      self.pub_vel.publish(vel_msg)
      print(vel_msg)

  def pan_cam(self):
      vel_msg = Twist()
      vel_msg.angular.z = -1.0
      self.pub_vel.publish(vel_msg)
      time.sleep(1)
      vel_msg.angular.z = 0
      self.pub_vel.publish(vel_msg)
      print(vel_msg)

  def follow_human(self):
    while True:
      human, weights = self.search_human()

      rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in human])
      pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
      for (xA, yA, xB, yB) in pick:
        cv2.rectangle(self.img, (xA, yA), (xB, yB), (0, 255, 0), 2)
      self.box_area.append(w * h)
      print("BOX AREA")
      print(self.box_area)
      self.move(sum(self.box_area)/len(self.box_area),(x + (x+w))/2)
      if len(self.box_area) > 2:
          # self.move(sum(self.box_area)/len(self.box_area),(x + (x+w))/2)
          self.box_area = []

      self.img = cv2.circle(self.img, ((x + (x+w))/2, (y + (y+h))/2), 2, (0, 0, 255), -1)

      img_2 = self.bridge.cv2_to_imgmsg(self.img, "bgr8")

      self.image_pub.publish(img_2)
      cv2.imshow('video', self.img)

      if cv2.waitKey(33) == 27:
          break

      cv2.destroyAllWindows()

  def move(self,area,side):
    vel_msg = Twist()
    if area < 100000:
      vel_msg.linear.x = abs(0.5)
    else:
      vel_msg.linear.x = -abs(0.25)

    if side < 400:
      vel_msg.angular.z = 0.25
    else:
      vel_msg.angular.z = -abs(0.25)

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    print(vel_msg)
    self.pub_vel.publish(vel_msg)
    time.sleep(1)
    vel_msg.linear.x = abs(0)
    vel_msg.angular.z = abs(0)
    self.pub_vel.publish(vel_msg)

  def run(self):
      r = rospy.Rate(20)
      global box_area,flag
      while not rospy.is_shutdown():
          r.sleep()
          self.follow_human()
          r.sleep()

if __name__ == '__main__':
    h_finder = image_converter()
    h_finder.run()
