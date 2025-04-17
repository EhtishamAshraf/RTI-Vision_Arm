# Python code to capture an Image with the Vimba camera 
# Note: when running this node, the roslaunch avt_vimba_camera mono_camera.launch should not be running nor the VimbaViewer

import cv2
from vimba import *
with Vimba.get_instance() as vimba :
    cams = vimba.get_all_cameras ()
    
    with cams[0] as cam:
        frame = cam.get_frame()
        frame.convert_pixel_format (PixelFormat.Mono8)
        cv2.imwrite ('frame_march.jpg ', frame.as_opencv_image ())
        print("Image saved successfully!")
        
    