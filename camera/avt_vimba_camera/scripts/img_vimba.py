#!/usr/bin/env python3

"""
ROS node to display, save and publish camera images to a random ROS topic

Note: when running this node, the roslaunch avt_vimba_camera mono_camera.launch should not be running nor the VimbaViewer
"""

import rospy
import cv2
from vimba import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def capture_and_publish():
    rospy.init_node('vimba_camera_node', anonymous=True)
    image_pub = rospy.Publisher('/vimba_camera/image_color', Image, queue_size=1)
    bridge = CvBridge()

    with Vimba.get_instance() as vimba: # create an instance of vimba sdk
        cams = vimba.get_all_cameras()
        if not cams:
            rospy.logerr("No Vimba cameras found!")
            return

        with cams[0] as cam:
            rospy.loginfo("Capturing image from Vimba camera...")
            
            frame = cam.get_frame()
            frame.convert_pixel_format(PixelFormat.Mono8)
            image = frame.as_opencv_image()                 # convert frame to opencv image
            resized_image = cv2.resize(image, (640, 480))

            original_size = image.shape
            rospy.loginfo(f"Original image size: {original_size}")

            resized_size = resized_image.shape
            rospy.loginfo(f"Resized image size: {resized_size}")

            image_path = 'captured_image_resized.jpg'
            cv2.imwrite(image_path, resized_image)
            rospy.loginfo(f"Resized image saved as {image_path}")

            cv2.imshow("Resized Captured Image", resized_image)
            cv2.waitKey(0) 
            cv2.destroyAllWindows()

            ros_image = bridge.cv2_to_imgmsg(resized_image, encoding="mono8")
            image_pub.publish(ros_image)
            rospy.loginfo("Resized image published!")

if __name__ == '__main__':
    try:
        capture_and_publish()
    except rospy.ROSInterruptException:
        pass
