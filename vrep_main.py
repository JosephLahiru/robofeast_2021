# -*- coding: utf-8 -*-
"""
Created on Sat Sep  4 13:54:04 2021

@author: Joseph Rasanjana
"""

import sim
from time import sleep as delay
import numpy as np
import cv2
import math
import sys

import threading

print ('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if (clientID != -1):
    print ('Connected to remote API server')

else:
    sys.exit('Failed connecting to remote API server')
    
blocking = sim.simx_opmode_blocking
oneshot = sim.simx_opmode_oneshot
streaming = sim.simx_opmode_streaming
buffering = sim.simx_opmode_buffer
oneshot_wait = sim.simx_opmode_oneshot_wait

def jog(target, step, handle):
    
    rc, cur_angle = sim.simxGetJointPosition(clientID, jointHandles[handle][1], buffering)
    cur_angle = round(math.degrees(cur_angle), 3)
    
    while(cur_angle != target):
        rc, cur_angle = sim.simxGetJointPosition(clientID, jointHandles[handle][1], buffering)
        cur_angle = round(math.degrees(cur_angle), 3)
        print(cur_angle)
        
        if(cur_angle>target):
            angle = cur_angle-step
        elif(cur_angle<target):
            angle = cur_angle+step
            
        rc = sim.simxSetJointPosition(clientID, jointHandles[handle][1], math.radians(angle), streaming)
            
def defa():
    jog(0, 1, 1)#base
    jog(0, 1, 2)#forward backward
    jog(0, 1, 3)#up down
    jog(0, 1, 4)#gripper left right tilt
    jog(0, 1, 5)#gripper up down

#errorCode, camera_handle1 = sim.simxGetObjectHandle(clientID, 'cam1', sim.simx_opmode_oneshot_wait)
errorCode, camera_handle2 = sim.simxGetObjectHandle(clientID, 'cam2', oneshot_wait)
errorCode, gripper_handle = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', oneshot_wait)
#errorCode, tip_handle = sim.simxGetObjectHandle(clientID, 'tip', oneshot_wait)

delay(0.25)
#returnCode, resolution1, image1 = sim.simxGetVisionSensorImage(clientID, camera_handle1, 0, streaming)
returnCode, resolution2, image2 = sim.simxGetVisionSensorImage(clientID, camera_handle2, 0, streaming)
delay(0.25)

jointHandles = []

for i in range(6):
    errorCode, jointHandles.append(sim.simxGetObjectHandle(clientID, f'IRB4600_joint{i}', oneshot_wait))

for i in range(6):
    rc, cur_angle = sim.simxGetJointPosition(clientID, jointHandles[i][1], streaming)

delay(.3)
print('staring')

template = cv2.imread('template.jpg', 0)
w, h = template.shape[::-1]
threshhold = 0.85

jog(90, 1, 4)#locked at 90 - Gripper
delay(0.1)

rc = sim.simxSetJointMaxForce(clientID, gripper_handle, 20, oneshot)
#rc = sim.simxSetJointTargetVelocity(clientID, gripper_handle, 0.5, streaming)

#table level control
#jog(-20, 1, 3)#up down
#jog(54, 1, 2)#forward backward

rc = sim.simxSetJointTargetVelocity(clientID, gripper_handle, 0.5, streaming)
delay(0.5)

#returnCode, tip_handle = sim.simxGetCollisionHandle(clientID,'tip', blocking)

#defa()

x = 68
y = -10
z = -4

x = 0
y = 0
z = 0

jog(0, 1, 1)#base

jog(0, 1, 3)#up down decrease
jog(0, 1, 2)#forward backward increase

while(0):
    
    ch = input("choose : ")
    
    if(ch == 'a'):
        x+=1
    if(ch == 's'):
        y+=1
    if(ch == 'd'):
        x-=1
    if(ch == 'w'):
        y-=1
    if(ch == 'g'):
        rc = sim.simxSetJointTargetVelocity(clientID, gripper_handle, -0.5, streaming)
        delay(0.25)
    if(ch == 'u'):
        rc = sim.simxSetJointTargetVelocity(clientID, gripper_handle, 0.5, streaming)
        delay(0.25)
    if(ch == 'r'):
        jog(0, 1, 4)#locked at 0 - Gripper
    if(ch == 'b'):
        break
    
    x1 = threading.Thread(target=lambda:jog(x, 1, 3)) # 75 68
    x2 = threading.Thread(target=lambda:jog(y, 1, 2)) # -18 -9 
    base = threading.Thread(target=lambda:jog(z, 1, 1))
    
    x1.start()
    x2.start()
    base.start()

#delay(5)
#rc = sim.simxSetJointTargetVelocity(clientID, gripper_handle, -0.5, streaming)

#jog(-60, 1, 1)#base
#delay(0.1)

while(0):
    #returnCode, resolution1, image1 = sim.simxGetVisionSensorImage(clientID, camera_handle1, 0, sim.simx_opmode_buffer)
    returnCode, resolution2, image2 = sim.simxGetVisionSensorImage(clientID, camera_handle2, 0, sim.simx_opmode_buffer)
    
    #im1 = np.array(image1, dtype=np.uint8)
    im2 = np.array(image2, dtype=np.uint8)
    
    #im1.resize([resolution1[0], resolution1[1], 3])
    im2.resize([resolution2[0], resolution2[1], 3])
    
    #im1 = cv2.flip(im1, 0)
    #im1 = cv2.resize(im1, (400, 400))
    #im1 = cv2.cvtColor(im1, cv2.COLOR_RGB2BGR)
    
    
    im2 = cv2.flip(im2, 0)
    im2 = cv2.resize(im2, (600, 600))
    im2 = cv2.cvtColor(im2, cv2.COLOR_RGB2BGR)
    
    im2_gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    
    res = cv2.matchTemplate(im2_gray, template, cv2.TM_CCOEFF_NORMED)
    loc = np.where(res >= threshhold)
    
    
    for pt in zip(*loc[::-1]):
        cv2.rectangle(im2, pt, (pt[0]+w, pt[1]+h), (0, 255, 0), 1)
        #cv2.putText(im2, str(num), (pt[0], pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
        print(pt)
        
    cv2.rectangle(im2, pt, (278+w, 406+h), (0, 0, 255), 1)
    
    #yellow one 278, 406

    
    #cv2.imshow("feed1", im1)
    cv2.imshow("feed2", im2)
    com = cv2.waitKey(100)
    if(com == ord('q')):
        break
    if(com == ord('c')):
        cv2.imwrite("template2.jpg", im2)

cv2.destroyAllWindows()