#!/usr/bin/env python3

"""
ROS node to read image data from the /camera/image_mono topic and adding a bounding box on the object.

Note: when running this node, the roslaunch avt_vimba_camera mono_camera.launch should be running in a separate terminal.
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VimbaImageProcessor:
    def __init__(self):
        rospy.init_node('vimba_image_processor', anonymous=True)

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/image_mono', Image, self.image_callback)

        rospy.loginfo("Subscribed to /camera/image_mono topic...")
        rospy.spin()  

    def image_callback(self, msg):
        rospy.loginfo("Received image, processing...")
        
        try:
            
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

            h, w = image.shape[:2]

            rospy.loginfo(f"Original image size: {h}x{w}")

            x1, y1 = int(w * 0.1), int(h * 0.1)  # Top-left corner
            x2, y2 = int(w * 0.7), int(h * 0.7)  # Bottom-right corner

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 5) 

            resized_image = cv2.resize(image, (640, 480))

            h_resized, w_resized = resized_image.shape[:2]
            rospy.loginfo(f"Resized image size: {h_resized}x{w_resized}")

            cv2.imshow('Processed Image with Bounding Box', resized_image)
            cv2.waitKey(1)  

            cv2.imwrite('processed_image_with_box_resized.jpg', resized_image)
            rospy.loginfo("Processed and resized image saved successfully!")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    try:
        VimbaImageProcessor()
    except rospy.ROSInterruptException:
        pass
