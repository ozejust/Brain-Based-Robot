# -*- coding: utf-8 -*-
"""
Created on Thu Jul  9 15:30:54 2015

@author: Pierre Jacquot
"""

import vrep,time,sys
import matplotlib.pyplot as plt
from PIL import Image as I
import array
import cv2
import numpy as np

frame11=[]
frame12=[]
def streamVisionSensor(visionSensorName,clientID,pause=0.0001):
    #Get the handle of the vision sensor
    res1,visionSensorHandle=vrep.simxGetObjectHandle(clientID,visionSensorName,vrep.simx_opmode_oneshot_wait)
    #Get the image
    res2,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_streaming)
    #Allow the display to be refreshed
    plt.ion()
    #Initialiazation of the figure
    time.sleep(0.5)
    res,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
    im = I.new("RGB", (resolution[0], resolution[1]), "white")
    #Give a title to the figure
    fig = plt.figure(1)    
    fig.canvas.set_window_title(visionSensorName)
    #inverse the picture
    plotimg = plt.imshow(im,origin='lower')
    #Let some time to Vrep in order to let him send the first image, otherwise the loop will start with an empty image and will crash
    time.sleep(1)
    while (vrep.simxGetConnectionId(clientID)!=-1): 
        #Get the image of the vision sensor
        res,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
        #Transform the image so it can be displayed using pyplot
        image_byte_array = array.array('b',image)
        im = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)

        #Update the image
        plotimg.set_data(im)
        #Refresh the display
        plt.draw()
        #The mandatory pause ! (or it'll not work)
        plt.pause(pause)
    print 'End of Simulation'
    
def getVisionSensor(visionSensorName,clientID):
    flag = False
    colour = 0
    if flag == False :
        #Get the handle of the vision sensor
        res1,visionSensorHandle=vrep.simxGetObjectHandle(clientID,visionSensorName,vrep.simx_opmode_oneshot_wait)
        #Get the image
        res2,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_streaming)
        frame1 = np.zeros((64,64))
        #hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        x = -1
        y = -1
        frame  = np.zeros((64,64))
        #time.sleep(1)
        flag = True
        
    if flag == True and (vrep.simxGetConnectionId(clientID)!=-1): 
        #Get the image of the vision sensor
        #time.sleep(0.01)
        err,resolution,image=vrep.simxGetVisionSensorImage(clientID,visionSensorHandle,0,vrep.simx_opmode_buffer)
    
        if err == vrep.simx_return_ok:
            image_byte_array = array.array('b',image)
            im = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
            im2 = np.asarray(im)
            im2 = cv2.flip(im2,flipCode=0)
        
            
#        img = np.array(image, dtype = np.uint8)
#        img.resize([resolution[0], resolution[1], 3])
#        img = np.rot90(img,2)
#        img = np.fliplr(img)
        #img = cv2.cvtColor(im2, cv2.COLOR_RGB2BGR)
        
        
             #Convert image to hsv
            frame1 =  cv2.cvtColor(im2, cv2.COLOR_RGB2BGR)
            
            
            hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
            #hsv = cv2.cvtColor(im2, cv2.COLOR_RGB2HSV)
            #Finding centroid by moments method
            lower_green = np.array([50,50,50], dtype=np.uint8)
            upper_green = np.array([70, 255, 255], dtype=np.uint8)
            mask_green = cv2.inRange(hsv, lower_green, upper_green) #Create mask
            
            lower_purple = np.array([135,50,50], dtype=np.uint8)
            upper_purple = np.array([165, 255, 255], dtype=np.uint8)
            mask_purple = cv2.inRange(hsv, lower_purple, upper_purple) 
            
            moments_green = cv2.moments(mask_green)
            area_green = moments_green['m00']
            if(area_green > 200):
                colour = 1
                x = int(moments_green['m10']/moments_green['m00'])
                y = int(moments_green['m01']/moments_green['m00'])
                #cv2.rectangle(img, (x, y), (x+2, y+2),(0,0,255), 2)
            
            moments_purple = cv2.moments(mask_purple)
            area_purple = moments_purple['m00']
            if(area_purple > 200):
                colour = 2
                x = int(moments_purple['m10']/moments_purple['m00'])
                y = int(moments_purple['m01']/moments_purple['m00'])
                #cv2.rectangle(img, (x, y), (x+2, y+2),(0,0,255), 2)
            frame = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            frame = cv2.resize(frame,(64,64))    
        
        else:
            pass
    
          
    return frame1,frame,colour,x,y  
    #print 'End of Simulation'
    
    cv2.destroyAllWindows()
"""    
if __name__ == '__main__':
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.2',19999,True,True,5000,5)
    if clientID!=-1:
        print 'Connected to remote API server'
        #Get and display the pictures from the camera
        #streamVisionSensor('NAO_vision1',clientID,0.0001)
        #Only get the image
        getVisionSensor('NAO_vision1',clientID)

    else:
        print 'Connection non successful'
        sys.exit('Could not connect')
"""

