
import sys
import vrep

import nengo

from AFRN import AFRN

from Synapse_VD import synapse_vd
from arborization import random_arbor



import time

import numpy as np

from vision_sensor import getVisionSensor



        
def nengo_network(clientID,dist_cbs):
    before_m = time.time()
    model = nengo.Network()
    #cap = cam_init(0)
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
    
    #plt.ion()
    
    with model: 
            # create network for webcam
            IT_line = np.load("IT_line.dat")
            IT_line = 0.15*IT_line
            IT_blob = np.load("IT_blob.dat")
            IT_blob = 0.15*IT_blob
            ce = np.load("ce.dat")
            cf = np.load("cf.dat")
            cf_ave = np.load("cf_ave.dat")
            cg = np.load("cg.dat")
            global taste_app
            global taste_ave
            taste_app = 0
            taste_ave = 0
            global time_app
            global time_ave
            time_app = 0
            time_ave = 0
            global flag_app
            global flag_ave
            flag_app = 0
            flag_ave = 0
            
            
            def key_app(t, im):
                
               global taste_app
               global time_app
               global flag_app
               
               t_now = time.time()
               delta_t_app = t_now - time_app
               _,_,dist1,_,dist3,dist4 = dist_cbs()
               
               if (dist1 <= 0.4 or dist3 <= 0.4 or dist4 <= 0.4) and flag_app == 0:
                   flag_app = 1
                   time_app = time.time()
                   
               
               if (dist1 <= 0.4 or dist3 <= 0.4 or dist4 <= 0.4) and delta_t_app < 50 and time_app != 0:
                   taste_app = 1
               else:
                   taste_app = 0
                   
               print "time_app: ", time_app
               print "flag_app: ", flag_app
               print "delta_t: ", delta_t_app
               
               if taste_app == 1 and taste_ave!=1:
                   return np.ones(18)
               else:
                   return np.zeros(18)
    
            def key_ave(t, im):
                
               global taste_ave
               global time_ave
               global flag_ave
               
               t_now = time.time()
               delta_t_ave = t_now - time_ave
               dist,dist0,_,dist2,_,_ = dist_cbs()
               
               if (dist <= 0.4 or dist0 <= 0.4 or dist2 <= 0.4) and flag_ave == 0:
                   flag_ave = 1
                   time_ave = time.time()
                   
               if (dist <= 0.4 or dist0 <= 0.4 or dist2 <= 0.4) and delta_t_ave < 50 and time_ave != 0:
                   taste_ave = 1
               else:
                    taste_ave = 0
                    
               if taste_ave == 1 and taste_app != 1:
                   return np.ones(18)
               else:
                   return np.zeros(18)

                
            def eyes_color(t,im):
                
                
#                before=time.time()
                _,_,colour,_,_ = getVisionSensor('NAO_vision1',clientID)
#                after=time.time()
#                ts=after-before
#                print "ts=", ts
                if colour==1:
                   return IT_blob.flatten()
                elif colour==2:
                    return IT_line.flatten()
                else:
                    return np.zeros(dim_IT)
               
            def eyes(t,im):
            
               
                frame1,frame,_,_,_ = getVisionSensor('NAO_vision1',clientID)
                #cv2.imshow('Stream',frame1)
                frame = frame.flatten()
                return frame/255.
            
             
                
#            R = nengo.Node(output=eyes,size_in=1,size_out=dim)
            
            V_C = nengo.Node(output=eyes_color,size_in=1,size_out=dim_IT)
            
            Tapp_in = nengo.Node(output=key_app,size_in=1,size_out=dim_T)
            Tave_in = nengo.Node(output=key_ave,size_in=1,size_out=dim_T)
            
            IT = nengo.Ensemble(n_neurons=dim_IT,dimensions=1
               ,neuron_type=AFRN(omega=0.03,g=1.1,thre=0.04)
               #,encoders=np.identity(dim)
               ,max_rates=80*np.ones(dim_IT)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            
            nengo.Connection(V_C,IT.neurons)
    
    
            """------------------------------------------------------------------"""
            
            Mapp = nengo.Ensemble(n_neurons=dim_M,dimensions=1
               ,neuron_type=AFRN(omega=0.30,g=2.0,thre=0.10)
               #,encoders=np.identity(dim)
               ,max_rates=[80]*np.ones(dim_M)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            
            Mave = nengo.Ensemble(n_neurons=dim_M,dimensions=1
               ,neuron_type=AFRN(omega=0.30,g=2.0,thre=0.10)
               #,encoders=np.identity(dim)
               ,max_rates=[80]*np.ones(dim_M)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            """------------------------------------------------------------------"""
            cij0_M_M_max=-0.05
            cij0_M_M_min=-0.12
            
            cij0_Mapp_Mave = (cij0_M_M_max - cij0_M_M_min) * np.random.rand(dim_M,dim_M) + cij0_M_M_min
            cij0_Mave_Mapp = (cij0_M_M_max - cij0_M_M_min) * np.random.rand(dim_M,dim_M) + cij0_M_M_min       
            
            nengo.Connection(Mapp.neurons,Mave.neurons,transform=np.multiply(np.ones((dim_M,dim_M)),cij0_Mapp_Mave))
            nengo.Connection(Mave.neurons,Mapp.neurons,transform=np.multiply(np.ones((dim_M,dim_M)),cij0_Mave_Mapp))
            """------------------------------------------------------------------"""
            
            S = nengo.Ensemble(n_neurons=dim_S,dimensions=1
               ,neuron_type=AFRN(omega=0.15,g=2.0,thre=0.05)
               #,encoders=np.identity(dim)
               ,max_rates=[80]*np.ones(dim_S)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            
            So = nengo.Ensemble(n_neurons=dim_So,dimensions=1
               ,neuron_type=AFRN(omega=0.22,g=3.0,thre=0.15)
               #,encoders=np.identity(dim)
               ,max_rates=[80]*np.ones(dim_So)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            
            Si = nengo.Ensemble(n_neurons=dim_Si,dimensions=1
               ,neuron_type=AFRN(omega=0.22,g=2.0,thre=0.10)
               #,encoders=np.identity(dim)
               ,max_rates=[80]*np.ones(dim_Si)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            
            Tapp = nengo.Ensemble(n_neurons=dim_T,dimensions=1
               ,neuron_type=AFRN(omega=0.30,g=2.0,thre=0.10)
               #,encoders=np.identity(dim)
               ,max_rates=[80]*np.ones(dim_T)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            
            Tave = nengo.Ensemble(n_neurons=dim_T,dimensions=1
               ,neuron_type=AFRN(omega=0.30,g=2.0,thre=0.10)
               #,encoders=np.identity(dim)
               ,max_rates=[80]*np.ones(dim_T)
            #,gain=10*np.ones(3600),bias=2*np.ones(3600)
            )
            
            """------------------------------------------------------------------"""
            nengo.Connection(Tapp_in,Tapp.neurons)
            nengo.Connection(Tave_in,Tave.neurons)
            """------------------------------------------------------------------"""
            nengo.Connection(Tapp.neurons,So.neurons,transform=0.12*cf)
            nengo.Connection(Tave.neurons,So.neurons,transform=0.12*cf_ave)
            """------------------------------------------------------------------"""
            nengo.Connection(Tapp.neurons,Mapp.neurons,transform=0.12*cg)
            nengo.Connection(Tave.neurons,Mave.neurons,transform=0.12*cg)
            """------------------------------------------------------------------"""
            cij0_Si_S_max=-0.30
            cij0_Si_S_min=-0.27
            
            cij0_So_S_max=0.11
            cij0_So_S_min=0.09
            
            cij0_So_Si_max=0.06
            cij0_So_Si_min=0.05
            
            cij0_Si_S = (cij0_Si_S_max - cij0_Si_S_min) * np.random.rand(dim_S,dim_Si) + cij0_Si_S_min
            cij0_So_S = (cij0_So_S_max - cij0_So_S_min) * np.random.rand(dim_S,dim_So) + cij0_So_S_min
            cij0_So_Si = (cij0_So_Si_max - cij0_So_Si_min) * np.random.rand(dim_Si,dim_So) + cij0_So_Si_min
            
            nengo.Connection(Si.neurons,S.neurons,transform=np.multiply(np.ones((dim_Si,dim_S)),cij0_Si_S))
            nengo.Connection(So.neurons,S.neurons,transform=np.multiply(ce,cij0_So_S))
            nengo.Connection(So.neurons,Si.neurons,transform=np.multiply(ce,cij0_So_Si))  
            """------------------------------------------------------------------"""
            IT_Mapp = random_arbor(dim_pre=dim_IT,dim_post=dim_M,p=0.15)
            IT_Mave1 = random_arbor(dim_pre=dim_IT,dim_post=dim_M,p=0.15)
            IT_Mave2 = random_arbor(dim_pre=dim_IT,dim_post=dim_M,p=0.15)
            
            IT_So = random_arbor(dim_pre=dim_IT,dim_post=dim_So,p=0.05)
            f_IT_Mapp = synapse_vd(theta1=0.01, theta2=0.16
                 , k1=0.1, k2=0.16, eta=0.006, eps=0.00006, rho=6.0
                 , w_rand_min=0.0006,  w_rand_max=0.0010,n_pre=dim_IT,n_post=dim_M,transform=IT_Mapp)
            f_IT_Mave_1 = synapse_vd(theta1=0.010, theta2=0.16
                 , k1=0.1, k2=0.16, eta=0.006, eps=0.00006, rho=6.0
                 , w_rand_min=0.0006,  w_rand_max=0.0010,n_pre=dim_IT,n_post=dim_M,transform=IT_Mave1)
            f_IT_Mave_2 = synapse_vd(theta1=0.005, theta2=0.08
                 , k1=0.1, k2=0.48, eta=0.006, eps=0.00006, rho=6.0
                 , w_rand_min=0.0006,  w_rand_max=0.0010,n_pre=dim_IT,n_post=dim_M,transform=IT_Mave2)
            f_IT_So = synapse_vd(theta1=0.01, theta2=0.18
                 , k1=0.05, k2=0.05, eta=0.02, eps=0.002, rho=6.0
                 , w_rand_min=0.0005,  w_rand_max=0.0015,n_pre=dim_IT,n_post=dim_So,transform=IT_So)
                 
            Syn_IT_Mapp=nengo.Node(output=f_IT_Mapp.func,size_in=dim_IT+dim_M+dim_S,size_out=dim_M)  
            Syn_IT_Mave_1=nengo.Node(output=f_IT_Mave_1.func,size_in=dim_IT+dim_M+dim_S,size_out=dim_M)  
            Syn_IT_Mave_2=nengo.Node(output=f_IT_Mave_2.func,size_in=dim_IT+dim_M+dim_S,size_out=dim_M)
            Syn_IT_So=nengo.Node(output=f_IT_So.func,size_in=dim_IT+dim_So+dim_S,size_out=dim_So)  
            
            nengo.Connection(IT.neurons,Syn_IT_Mapp[0:dim_IT],transform= np.identity(dim_IT))  
            nengo.Connection(Mapp.neurons,Syn_IT_Mapp[dim_IT:dim_IT+dim_M],transform= np.identity(dim_M))
            nengo.Connection(S.neurons,Syn_IT_Mapp[3::-1],transform= np.identity(dim_S))
            nengo.Connection(Syn_IT_Mapp[0:dim_M],Mapp.neurons,transform= np.identity(dim_M)) 
            
            nengo.Connection(IT.neurons,Syn_IT_Mave_1[0:dim_IT],transform= np.identity(dim_IT))  
            nengo.Connection(Mave.neurons,Syn_IT_Mave_1[dim_IT:dim_IT+dim_M],transform= np.identity(dim_M))
            nengo.Connection(S.neurons,Syn_IT_Mave_1[3::-1],transform= np.identity(dim_S))
            nengo.Connection(Syn_IT_Mave_1[0:dim_M],Mave.neurons,transform= np.identity(dim_M)) 
            
            nengo.Connection(IT.neurons,Syn_IT_Mave_2[0:dim_IT],transform= np.identity(dim_IT))  
            nengo.Connection(Mave.neurons,Syn_IT_Mave_2[dim_IT:dim_IT+dim_M],transform= np.identity(dim_M))
            nengo.Connection(S.neurons,Syn_IT_Mave_2[3::-1],transform= np.identity(dim_S))
            nengo.Connection(Syn_IT_Mave_2[0:dim_M],Mave.neurons,transform= np.identity(dim_M)) 
            
            nengo.Connection(IT.neurons,Syn_IT_So[0:dim_IT],transform= np.identity(dim_IT))  
            nengo.Connection(So.neurons,Syn_IT_So[dim_IT:dim_IT+dim_So],transform= np.identity(dim_So))
            nengo.Connection(S.neurons,Syn_IT_So[3::-1],transform= np.identity(dim_S))
            nengo.Connection(Syn_IT_So[0:dim_So],So.neurons,transform= np.identity(dim_So)) 
            """------------------------------------------------------------------"""
                    #eye_b_probe = nengo.Probe(VA_H.neurons, 'output', synapse=0.0)
            #R_probe= nengo.Probe(R, 'output', synapse=0.0)      
            V_C_probe= nengo.Probe(V_C, 'output', synapse=0.0)
            IT_probe= nengo.Probe(IT.neurons, 'output', synapse=0.0)
            Mapp_probe= nengo.Probe(Mapp.neurons, 'output', synapse=0.0)
            Mave_probe= nengo.Probe(Mave.neurons, 'output', synapse=0.0)
            Tapp_probe= nengo.Probe(Tapp.neurons, 'output', synapse=0.0)
            Tave_probe= nengo.Probe(Tave.neurons, 'output', synapse=0.0)
            S_probe= nengo.Probe(So.neurons, 'output', synapse=0.0)
            """------------------------------------------------------------------"""
    after_m = time.time()        
    t_m = after_m - before_m
    print "time for model build : ", t_m
            
    sim = nengo.Simulator(model,dt=0.01)
    
    #return sim, R_probe, V_C_probe, IT_probe, Mapp_probe, Mave_probe, Tapp_probe, Tave_probe, S_probe 
    return sim, V_C_probe, IT_probe, Mapp_probe, Mave_probe, Tapp_probe, Tave_probe, S_probe 
