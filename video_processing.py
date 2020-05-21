#!/usr/bin/env python3

# Requires: 
# sudo apt install v4l-utils
# pip3 install python-opencv
import numpy as np
import cv2
import os
import serial

video_srcL = 0
camera_serial_port = "/dev/ttyUSB0"

#video_srcR = 1



print(" __      _       __              _____ _ ")
print(" \ \    / (_)   | |             / ____| |   ")
print("  \ \  / / _  __| | ___  ___   | |  __| | __ _ ___ ___  ___  ___  ")
print("   \ \/ / | |/ _` |/ _ \/ _ \  | | |_ | |/ _` / __/ __|/ _ \/ __| ")
print("    \  /  | | (_| |  __/ (_) | | |__| | | (_| \__ \__ \  __/\__ \ ")
print("     \/   |_|\__,_|\___|\___/   \_____|_|\__,_|___/___/\___||___/ ")

print(" ...for James")
print(" Wash in the pool of Siloam")

try:
    serial_enabled = True
    ser = serial.Serial(camera_serial_port, baudrate=57600,  timeout=0) 
except:
    print("Could not enable serial port on {}".format(camera_serial_port))
    serial_enabled = False


#print(" Right video source: camera # "+str(video_srcR))
#print(" Left video source: camera # "+str(video_srcL))

# Open video stream
camL = cv2.VideoCapture(int(video_srcL))
#camR = cv2.VideoCapture(int(video_srcR))


print(camL.set(cv2.CAP_PROP_FRAME_HEIGHT, 320))
print(camL.set(cv2.CAP_PROP_FRAME_HEIGHT, 240))
#

os.system("v4l2-ctl -d /dev/video0 -i 0 -s 0 --set-fmt-video=width=320,height=240,pixelformat=4")

#print(camR.set(cv2.CAP_PROP_FRAME_HEIGHT, 600.0))
#print(camR.set(cv2.CAP_PROP_FRAME_WIDTH, 800.0))

#print camL.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
#print camL.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
 
#camR = cv2.VideoCapture(video_srcR)

#right_frame = camR.read()

#cv2.namedWindow('videobox')

showedge = 0
edge_thrsh_1 = 1500
edge_thrsh_2 = 2000

def nothing(value):
    pass


def agc(value):
    cmd = "AGC:{:d}".format(value)
    camCmd(cmd)

def moc(value):
    cmd = "MOC:{:d}".format(value)
    camCmd(cmd)

def gain(value):
    cmd = "MGC:{:d}".format(value)
    camCmd(cmd)

def senstog(value):
    cmd = "SENSTOG:{:d}".format(value)
    camCmd(cmd)

def focuspos(value):
    cmd = "FOCUSPOS:{:d}".format(value)
    camCmd(cmd)


def fov(value):
    
    if (value == 0):
        cmd = "FOVW:"
    elif (value == 1):
        cmd = "FOVM:"
    elif (value == 2):
        cmd = "FOVN:"

    camCmd(cmd)

def cal1(value):
    cmd = "CAL1:"
    camCmd(cmd)
    #cv2.setTrackbarPos('Calibrate', 'controls', 0)

def af(value):
    cmd = "AF:"
    camCmd(cmd)
    #cv2.setTrackbarPos('AutoFocus', 'controls', 0)

def camCmd(cmd):
    cmd = cmd + "\015"
    print(cmd)
    ser.write(cmd.encode('ascii'))




# Video window
window_width = 1260
window_height = int((float(3/4) * float(window_width)))
cv2.namedWindow('controls', flags=cv2.WINDOW_NORMAL)
cv2.resizeWindow('controls', window_width, window_height)

# Controls for camera
#cv2.createButton("button5",senstog,None,cv2.CV_RADIOBOX,0)

if serial_enabled:
    cv2.createTrackbar('AutoFocus', 'controls', 0, 1, af)
    cv2.createTrackbar('Focus', 'controls', 0, 4096, focuspos)
    cv2.createTrackbar('Calibrate', 'controls', 0, 1, cal1)
    cv2.createTrackbar('Sensitivity', 'controls', 0, 1, senstog)
    cv2.createTrackbar('AGC', 'controls', 0, 1, agc)
    cv2.createTrackbar('Offset', 'controls', 0, 4096, moc)
    cv2.createTrackbar('Gain', 'controls', 0, 4096, gain)
    cv2.createTrackbar('FOV', 'controls', 0, 2, fov)




while True:
    #ret, right_frame = camR.read()     # Nab a frame of video stream
    ret, left_frame = camL.read()
    #print camL.get(cv2.cv.CV_CAP_PROP_FPS)
    
    #right_frame = np.zeros_like(left_frame)   # There is no right camera yet :-(
    
    #thrs1 = cv2.getTrackbarPos('thrs1', 'sliders')
    #thrs2 = cv2.getTrackbarPos('thrs2', 'sliders')

    #right_frame = cv2.Canny(right_frame, edge_thrsh_1, edge_thrsh_2, apertureSize=5)
    #left_frame = cv2.Canny(left_frame, edge_thrsh_1, edge_thrsh_2, apertureSize=5)

    #if(showedge):
    #    #right_edge = cv2.Canny(right_frame, edge_thrsh_1, edge_thrsh_2, apertureSize=5)
    #left_edge = cv2.Canny(left_frame, edge_thrsh_1, edge_thrsh_2, apertureSize=5)
    
    #    #right_frame[right_edge != 0] = (0,255,255)
    #left_frame[left_edge != 0] = (0,255,255)
    
    #right_frame = np.split(right_frame, 2, axis=1)[0]
    #left_frame = np.split(left_frame, 2, axis=1)[0]
    
    focal_length = 600; # This is the focal length of the lens, measured in pixels
    x_offset = 800/2;
    y_offset = 600/2;
    
    cameraMatrix = np.matrix([[focal_length, 0, x_offset], [0, focal_length, y_offset], [0, 0, 1]])
    
    #vertical_smear_right = -0.1; # Variable names imply I haven't a clue what I'm doing yet
    vertical_smear_left = 0.1;
    
    #horizontal_smear_right = 0;
    horizontal_smear_left = 0;
    
    #fish_eye_ness_right = 0.2;
    fish_eye_ness_left = 0.2;
    
    #smaller_fish_eye_ness_right = 8;
    smaller_fish_eye_ness_left = 8;
    
    #distCoeffs_right = np.matrix([fish_eye_ness_right, smaller_fish_eye_ness_right, vertical_smear_right, horizontal_smear_right])
    distCoeffs_left = np.matrix([fish_eye_ness_left, smaller_fish_eye_ness_left, vertical_smear_left, horizontal_smear_left])
    
    #right_frame = cv2.undistort(right_frame, cameraMatrix, distCoeffs_right)
    #left_frame = cv2.undistort(left_frame, cameraMatrix, distCoeffs_left)
    
    #right_frame = np.rot90(right_frame, 3) # Fix rotation issues
    #left_frame = np.rot90(left_frame, 1)
    
    #video_out = np.concatenate((left_frame, right_frame), axis=1)
    video_out = left_frame

    cv2.imshow('controls', video_out)
    
    feedback = ser.read(64)

    if(feedback):
        print(str(feedback))

    ch = 0xFF & cv2.waitKey(1)

    if ch == ord('e'):
        showedge = not showedge

cv2.destroyAllWindows() 
