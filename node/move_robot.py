#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

 
class image_converter:

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
   

    vel_msg = Twist()

    grayScale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Threshold the frame to get only the colors in the defined range
    ret, mask = cv2.threshold(grayScale,140,255,cv2.THRESH_BINARY_INV)

    

    croppedMask = mask[700:]
    targetx = 400
    vel_msg.linear.x = 0.1

    # Find the contours in the mask
    contours, _ = cv2.findContours(croppedMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    currentx = 0

    # Draw the contours on the original frame
    if (contours):
      c = max(contours, key=cv2.contourArea)
      (x, y), radius = cv2.minEnclosingCircle(c)
      
      
      
      
      center = (int(x), int(y)+700)
      radius = int(int(radius))
      cv2.circle(frame, center, radius, (0, 255, 0), 2)

      error = x - targetx
      vel_msg.angular.z = -error * 0.03
      if radius>30:
        vel_msg.linear.x = 150/radius + 0.1
      else:
        vel_msg.linear.x = 0.1
      self.cmd_vel_pub.publish(vel_msg)

      
    


    cv2.imshow("Image window", frame)
    cv2.waitKey(2)

    # try:
    #   rospy.Rate(15).sleep() 
    #   self.image_pub.publish(vel_msg)
    # except CvBridgeError as e:
    #   print(e)
 
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
 
def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 