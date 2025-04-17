# Python script to print different properties of the camera and capture an image
# Note: when running this node, the roslaunch avt_vimba_camera mono_camera.launch should not be running nor the VimbaViewer

from vimba import *
import cv2

def print_preamble():
    print('//////////////////////////////////////')
    print('/// Vimba API List Cameras Example ///')
    print('//////////////////////////////////////\n')

def print_camera(cam: Camera):
    print('/// Camera Name   : {}'.format(cam.get_name()))
    print('/// Model Name    : {}'.format(cam.get_model()))
    print('/// Camera ID     : {}'.format(cam.get_id()))
    print('/// Serial Number : {}'.format(cam.get_serial()))
    print('/// Interface ID  : {}\n'.format(cam.get_interface_id()))

def main():
    print_preamble()
    with Vimba.get_instance() as vimba:
        cams = vimba.get_all_cameras()

        print('Cameras found: {}'.format(len(cams)))

        for cam in cams:
            print_camera(cam)
        
        print("camera is: ", cams[0])
        with cams[0] as cam:
            print("cam is actually: ", cam)
            frame = cam.get_frame()
            print("frame is: ", frame)
            frame.convert_pixel_format (PixelFormat.Mono8)
            print("Now frame is: ", frame)
            cv2.imwrite ('detected_img.jpg ', frame.as_opencv_image ())

if __name__ == '__main__':
    main()
