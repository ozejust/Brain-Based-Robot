import numpy as np
import random
import nengo
#import ModBCM


def random_arbor(dim_pre,dim_post,p):
    arbor = []
    for i in range(0,dim_post ):
        for j in range(0,dim_pre):

              prob = random.random()
              if prob > p:
                  arbor.extend([1.])
              else:
                  arbor.extend([0.])
    arbor = np.reshape(arbor,(dim_post,dim_pre))
    return arbor
   
def rec_arbor(pre,post,dim_pre,dim_post,h,w):
    arbor = np.ones((h,w))
    a = range(0,dim_post)
    a = np.reshape(a,(np.sqrt(dim_pre),np.sqrt(dim_pre)))
    a = a[:,:np.sqrt(dim_pre)-w]
    a = a.flatten()
    b = range(0,dim_post)
    b = np.reshape(b,(np.sqrt(dim_pre),np.sqrt(dim_pre)))
    b = b[:np.sqrt(dim_pre)-h,:]
    b = b.flatten()
    for j in b:
            for i in a:
                prek= [pre[k] for k in [0,5,10,15]]
                postk = [post[k] for k in [0,5,10,15]]
                nengo.Connection(prek,postk,transform = arbor
                #,solver = nengo.solvers.LstsqL2(weights=True)
                #,learning_rule_type=ModBCM(theta1=0.1, theta2=0.25, k1=0.45, k2=0.45, eta=0.125, eps=0.00125, rho=6.0 , w_rand_min=0.02,  w_rand_max=0.03)
                )
    return

def circ_arbor(pre,post,dim_pre,dim_post,r):
    arbor = []    
    for k in range(0,r):
        arbor.extend(np.zeros(r-k-1))
        arbor.extend(np.ones(k+1))
    arbor = np.reshape(arbor,(r,r))
    arbor = np.vstack([arbor,np.rot90(arbor)])
    arbor = np.hstack([arbor,np.fliplr(arbor)])
    #for i in range(0,):
        
    return arbor
    
def donut_arbor(pre,post,dim_pre,dim_post,ri,ro):
    arbor = []
    arbor1 = []
    for j in range(0,ro):
        arbor.extend(np.zeros(ro-j-1))
        arbor.extend(np.ones(j+1))
    arbor = np.reshape(arbor,(ro,ro))
    for i in range(0,ri):
        arbor1.extend(np.zeros(ri-i-1))
        arbor1.extend(np.ones(i+1))
    arbor1 = np.reshape(arbor1,(ri,ri))
    temp = np.zeros((ro,ro))
    temp[ro-ri:ro,ro-ri:ro]=arbor1
    arbor = arbor - temp
    arbor = np.vstack([arbor,np.rot90(arbor)])
    arbor = np.hstack([arbor,np.fliplr(arbor)])
    return arbor