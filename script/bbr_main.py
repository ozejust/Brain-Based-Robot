
import sys
import vrep

from naoqi import ALProxy
from manage_joints import get_first_handles,JointControl

import threading
import time
import cv2
import numpy as np
from vision_sensor import getVisionSensor
from nengo_network import nengo_network

greenLower = (53,55,55) 
greenUpper = (90,255,255)
pinkLower = (155,100,100) 
pinkUpper = (180,255,255)
hdirection = 0.0
flag_app = 0
flag_ave = 0

def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints

    proxy.setSmartStiffnessEnabled(False)

    proxy.rest()
    proxy.wakeUp()

    proxy.setFallManagerEnabled(True)
    #print proxy.getSummary()


class Robot:
    def __init__(self, motionProxy, postureProxy, vrepclientID):
        self.mp = motionProxy
        self.pp = postureProxy
        self.RawPosition = None
        self.RawOrientation = None
        self.DiscretePosition = None
        self.vrepclientID = vrepclientID
        self.firstPosVerification = True
        self.firstOrVerification = True
        self.firstMsgVerification = True
        self.lastOrientation = 'R'
        self.hdirection = hdirection
        self.flag_app = flag_app
        self.flag_ave = flag_ave
    def WalkConfigs(self):
        #####################
        ## FOOT CONTACT PROTECTION
        #####################
        #~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
        self.mp.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        #####################
        ## Enable arms control by move algorithm
        #####################
        self.mp.setMoveArmsEnabled(True, True)
        #time.sleep(5)
    

    def GetRawPosition(self):
        #####################
        ## get robot position in the WORLD_FRAME and discretizes it on a labirinth Position
        #####################
        retries = 0
        while True:
            returnCode, NaoHandle = vrep.simxGetObjectHandle(self.vrepclientID,'NAO',vrep.simx_opmode_oneshot_wait)
            retries +=1
            if (returnCode == 0 and NaoHandle is not None) or retries > 3:
                break
            retries = 0
        if returnCode == 0:
            while True:
                if self.firstPosVerification:
                    returnCode, self.RawPosition =vrep.simxGetObjectPosition(clientID, NaoHandle ,-1 , vrep.simx_opmode_streaming )
                    self.firstPosVerification = False
                else:
                    returnCode, self.RawPosition =vrep.simxGetObjectPosition(clientID, NaoHandle ,-1 , vrep.simx_opmode_buffer )
                if (returnCode == 0 and self.RawPosition is not None) or retries > 3:
                    break
            
        print 'return code [%s] and position %s' %(returnCode, self.RawPosition)
        return self.RawPosition

    def GetRawOrientation(self):
        #####################
        ## get robot position in the WORLD_FRAME and discretizes it on a labirinth Position
        #####################
        retries = 0
        while True:
            returnCode, NaoHandle = vrep.simxGetObjectHandle(self.vrepclientID,'NAO',vrep.simx_opmode_oneshot_wait)
            retries +=1
            if (returnCode == 0 and NaoHandle is not None) or retries > 3:
                break
            retries = 0
        if returnCode == 0:
            while True:
                if self.firstOrVerification:
                    returnCode, self.RawOrientation =vrep.simxGetObjectOrientation(clientID, NaoHandle ,-1 , vrep.simx_opmode_streaming )
                    self.firstOrVerification = False
                else:
                    returnCode, self.RawOrientation =vrep.simxGetObjectOrientation(clientID, NaoHandle ,-1 , vrep.simx_opmode_buffer )
                if (returnCode == 0 and self.RawOrientation is not None) or retries > 3:
                    break
            
        print 'return code [%s] and position %s' %(returnCode, self.RawOrientation)
        return self.RawOrientation

    
    def CorrectSteps(self,step_x, step_y,nextPosition):
        current = self.GetRawPosition()
        print 'next %s' % nextPosition
        targetPoint = labirinthData[nextPosition[1]][nextPosition[0]]
        #print 'Positions: current [%s] | target [%s]' % (current, targetPoint)        
        gradx = targetPoint[0] - current[0]
        grady = targetPoint[1] - current[1]
        
        step_x = gradx
        step_y = grady
        return step_x, step_y
        
    def dist_cbs(self):
        ## Read position of blocks from V-REP then send to remoteAPI ##
        
        if self.firstMsgVerification:
            err,pos_str = vrep.simxGetStringSignal(clientID,'cuboidpos',vrep.simx_opmode_streaming)
            self.firstMsgVerification=False
        else:
            err,pos_str = vrep.simxGetStringSignal(clientID,'cuboidpos',vrep.simx_opmode_buffer)
        pos = vrep.simxUnpackFloats(pos_str)
        #for i in range(0,6):
        #dist2 = pos[2]
        
#        print "pos= ",pos
        if len(pos)==6:
            dist  = pos[0]
            dist0 = pos[1]
            dist1 = pos[2]
            dist2 = pos[3]
            dist3 = pos[4]
            dist4 = pos[5]
        else:
            dist  = 1.0
            dist0 = 1.0
            dist1 = 1.0
            dist2 = 1.0
            dist3 = 1.0
            dist4 = 1.0
            
        return dist,dist0,dist1,dist2,dist3,dist4
        
    
    def move(self,app,ave,Tapp,Tave,dist_cbs):

        #self.app = app
        #self.ave = ave
            
        
            
        #def turn_to_cube():
        frame1,_,_,x,y = getVisionSensor('NAO_vision1',clientID)
        dist,dist0,dist1,dist2,dist3,dist4 = dist_cbs()
        if dist <= 0.4 or dist0 <= 0.4 or dist1 <= 0.4 or dist2 <= 0.4 or dist3 <= 0.4 or dist4 <= 0.4:
            stop = 0
        else:
            stop = 1
        if x!=-1:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
            
            if (x< 22):
                direction = 1
                forward = 0
            elif (x> 40):
                direction = -1
                forward = 0
            else:
                direction = 0
                forward = 1
     
        else:
            direction = 0
            forward = 0
            
        self.mp.move(0.09*forward*stop,0,0.13*direction)
        
        if y!=-1:
            if (y< 29 and self.hdirection >= -0.62):
                self.hdirection -= 0.05
            elif (y> 35 and self.hdirection <= 0.43):
                self.hdirection += 0.05
            else:
                pass
            
        if (app > 0.6 and app > ave) and (Tapp < 0.1) and self.flag_app==0:
            #if dist < 0.4:
                self.mp.move(0,0,0)
                self.pp.goToPosture('Crouch', 0.5)
                self.mp.setAngles("HeadPitch", 0.4, 0.5)
                time.sleep(0.5)
                self.mp.setAngles("HeadPitch", -0.4, 0.5)
                time.sleep(0.5)
                self.mp.setAngles("HeadPitch", 0.4, 0.5)
                time.sleep(0.5)
                self.mp.setAngles("HeadPitch", -0.4, 0.5)
                time.sleep(0.5)
                self.flag_app = 1

        elif (ave > 0.6 and ave > app) and (Tave < 0.1) and self.flag_ave==0: 
                self.mp.move(0,0,0)
                self.pp.goToPosture('Crouch', 0.5)    
                self.mp.setAngles("HeadYaw", 0.4, 0.5)
                time.sleep(0.5)
                self.mp.setAngles("HeadYaw", -0.4, 0.5)
                time.sleep(0.5)
                self.mp.setAngles("HeadYaw", 0.4, 0.5)
                time.sleep(0.5)
                self.mp.setAngles("HeadYaw", -0.4, 0.5)
                time.sleep(0.5)
                self.flag_ave = 1
               
        if self.flag_app == 1 or self.flag_ave == 1:
            self.mp.move(0.0,0,0.2)
            self.mp.setAngles("HeadPitch", 0.0, 0.5)
            self.mp.setAngles("HeadYaw", 0.0, 0.5)
            time.sleep(4)
            self.flag_app = 0
            self.flag_ave = 0
        #if (msg == 'ave'):
            
        print ("Tapp: %f , Tave: %f "%(Tapp,Tave))
        print ("Mapp: %f , Mave: %f "%(app,ave))
#        else:
#            self.hdirection = 0
        
        self.mp.setAngles("HeadPitch", self.hdirection, 0.3)

class StoppableThread (threading.Thread):
    def __init__(self, clientID, motionProxy):
        super(StoppableThread, self).__init__()
        self.clientID = clientID
        self.motionProxy = motionProxy
        self.exitflag = threading.Event()
    def run(self):
        print "================ Handles Initialization ================"
        Head_Yaw=[];        Head_Pitch=[];
        L_Hip_Yaw_Pitch=[]; L_Hip_Roll=[];      L_Hip_Pitch=[]; L_Knee_Pitch=[];    L_Ankle_Pitch=[];    L_Ankle_Roll=[];
        R_Hip_Yaw_Pitch=[]; R_Hip_Roll=[];      R_Hip_Pitch=[]; R_Knee_Pitch=[];    R_Ankle_Pitch=[];    R_Ankle_Roll=[];
        L_Shoulder_Pitch=[];L_Shoulder_Roll=[]; L_Elbow_Yaw=[]; L_Elbow_Roll=[];    L_Wrist_Yaw=[]
        R_Shoulder_Pitch=[];R_Shoulder_Roll=[]; R_Elbow_Yaw=[]; R_Elbow_Roll=[];    R_Wrist_Yaw=[]
        R_H=[];             L_H=[];             R_Hand=[];      L_Hand=[];
        Body = [Head_Yaw,Head_Pitch,L_Hip_Yaw_Pitch,L_Hip_Roll,L_Hip_Pitch,L_Knee_Pitch,L_Ankle_Pitch,L_Ankle_Roll,R_Hip_Yaw_Pitch,
                R_Hip_Roll,R_Hip_Pitch,R_Knee_Pitch,R_Ankle_Pitch,R_Ankle_Roll,L_Shoulder_Pitch,L_Shoulder_Roll,L_Elbow_Yaw,L_Elbow_Roll,
                L_Wrist_Yaw,R_Shoulder_Pitch,R_Shoulder_Roll,R_Elbow_Yaw,R_Elbow_Roll,R_Wrist_Yaw,L_H,L_Hand,R_H,R_Hand]
    
        get_first_handles(clientID,Body)

        while(vrep.simxGetConnectionId(clientID)!=-1):
            if self.exitflag.isSet():
                print 'End of simulation'
                break

            JointControl(self.clientID,self.motionProxy,0,Body)
            #sleep(0.005)
    def SetExitFlag(self):
        self.exitflag.set()
        
def startingComunication(clientID, motionProxy):
    try:
        thread1 = StoppableThread(clientID,motionProxy)
        thread1.start()

    except:
       print "Error: unable to start process"
       sys.exit(1)
    return thread1


def main(robotIP,clientID):
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
        motionProxy.moveInit()
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e
        exit(1)
    
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e
        exit(1)


    # Set NAO in stiffness On
    StiffnessOn(motionProxy)

    nao = Robot(motionProxy, postureProxy,clientID)

    print '========== Setting Start Position =========='
    # Making some configs to start in the correct point and to walk smoothly

    #starting comunication between vrep and nao-qi
    thread1 = startingComunication(clientID, motionProxy)
    time.sleep(5)
    
    nao.WalkConfigs()

    print '========== NAO is listening =========='


    #path = retrieve_path(startPosition)
    print "================ follow the path ================"
    #nao.mp.move(0.15,0.0,0.0)
    before = time.time()
    
    sim, V_C_probe, IT_probe, Mapp_probe, Mave_probe, Tapp_probe, Tave_probe, S_probe = nengo_network(clientID,nao.dist_cbs)
    i = 0
    while (True):

        
        before = time.time()
        sim.run(0.01, progress_bar=False)
        
        h_crop = 64
        v_crop = 64
        dim = h_crop*v_crop
        h_det=4
        v_det=1
        h_IT=28
        w_IT=28
        dim_IT=h_IT*w_IT
        h_ITi=14
        w_ITi=14
        dim_ITi=h_ITi*w_ITi
        h_M,w_M=3,6
        dim_M=h_M*w_M
        h_S,w_S=2,2
        dim_S=h_S*w_S
        h_Si,w_Si=2,2
        dim_Si=h_Si*w_Si
        h_So,w_So=4,4
        dim_So=h_So*w_So
        h_T,w_T=3,6
        dim_T=h_T*w_T
        
        b1 = sim.data[V_C_probe]
        d1 = sim.data[IT_probe]
        e1 = sim.data[Mapp_probe]
        e2 = sim.data[Mave_probe]
        f1 = sim.data[Tapp_probe]
        f2 = sim.data[Tave_probe]
        g1 = sim.data[S_probe]

        b = b1[i:i+1,0:dim]
        d = d1[i:i+1,0:dim_IT]
        e1= e1[i:i+1,0:dim_M]
        e2= e2[i:i+1,0:dim_M]

        app = np.average(e1)
        ave = np.average(e2)
        
        f1= f1[i:i+1,0:dim_T]
        f2= f2[i:i+1,0:dim_T]

        Tapp = np.nanmean(f1)
        Tave = np.nanmean(f2)
        
        
        g = g1[i:i+1,0:dim_So]
        e1= np.reshape(e1,(h_M,w_M))
        e2= np.reshape(e2,(h_M,w_M))
        f1= np.reshape(f1,(h_T,w_T))
        f2= np.reshape(f2,(h_T,w_T))
        g = np.reshape(g,(h_So,w_So))
        e = np.hstack([e1,e2])
        f = np.hstack([f1,f2])
        

        b = np.reshape(b,(h_IT,w_IT))
        b = cv2.resize(b,(100,100))
        d = np.reshape(d,(h_IT,w_IT))
        d = cv2.resize(d,(400,400),interpolation=cv2.INTER_AREA)

        g = np.reshape(g,(h_So,w_So))
        g = cv2.resize(g,(100,100),interpolation=cv2.INTER_AREA)
        i+=1

        d = d*255
        e = e*255
        f = f*255
        g = g*255
        d = np.uint8(d)
        e = np.uint8(e)
        f = np.uint8(f)
        g = np.uint8(g)

        d = cv2.applyColorMap(d,cv2.COLORMAP_JET)
        
        e = cv2.applyColorMap(e,cv2.COLORMAP_JET)
        e = cv2.resize(e,(400,200),interpolation=cv2.INTER_AREA)
        f = cv2.applyColorMap(f,cv2.COLORMAP_JET)
        f = cv2.resize(f,(400,200),interpolation=cv2.INTER_AREA)
        g = cv2.applyColorMap(g,cv2.COLORMAP_JET)
        g = cv2.resize(g,(100,100),interpolation=cv2.INTER_AREA)
        
        
        cv2.imshow('V_C',b)
        cv2.imshow('Visual System',d)
        cv2.imshow('Motor',e)
        cv2.imshow('Taste',f)
        cv2.imshow('Value',g)
        nao.move(app,ave,Tapp,Tave,nao.dist_cbs)
        after = time.time()
        t = after - before

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break



    print "================ Closing Comunication ================"
    thread1.SetExitFlag()
    thread1.join()



if __name__ == "__main__":
    #robotIp = "127.0.0.1"   #run on local host (same PC)
    robotIp = "192.168.1.17" #run on an external pc

    if len(sys.argv) <= 1:
        print "Usage python motion_moveTo.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    print '================ Program Sarted ================'
    
    vrep.simxFinish(-1)
    #clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    clientID=vrep.simxStart('192.168.1.17',19999,True,True,5000,5)
    if clientID!=-1:
        print 'Connected to remote API server'
    
    else:
        print 'Connection non successful'
        sys.exit('Could not connect')
    #setStartPosition(clientID)
    main(robotIp, clientID)
    print "================ Exiting Session ================"
    
    vrep.simxFinish(-1)
