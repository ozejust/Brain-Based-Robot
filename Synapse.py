import numpy as np

class synapse:
    def __init__(self,theta1=0.1, theta2=0.25
             , k1=0.45, k2=0.45, eta=0.125, eps=0.00125, rho=6.0
             , w_rand_min=0.02,  w_rand_max=0.03,n_pre=9,n_post=9,transform=0.):
        self.theta1 = theta1
        self.theta2 = theta2
        self.k1 = k1 
        self.k2 = k2
        self.eta = eta
        self.eps = eps 
        self.rho = rho 
        self.n_pre = n_pre
        self.n_post = n_post
        self.transform = transform
        
        weights_init = (w_rand_max - w_rand_min) * np.random.rand(n_post,n_pre) + w_rand_min
        self.weights_init = weights_init
        self.cij = weights_init
    def func(self,t,x):
        
        theta1 = self.theta1
        theta2 = self.theta2
        k1 = self.k1
        k2 = self.k2
        eta = self.eta
        eps = self.eps
        rho = self.rho
        n_pre = self.n_pre
        n_post = self.n_post
        weights_init = self.weights_init
        transform = self.transform
        sj=x[0:n_pre]
        si=x[n_pre:n_post+n_pre]
        F = np.zeros(n_post)
        H = np.zeros((n_post,n_pre))
        

        for i in range(0,n_post):
            
            if (si[i] < theta1):
                F[i] = 0.0
                #print "case 1"
            elif (si[i] >= theta1 and si[i] < (theta1+theta2)/2) :
                F[i] = k1*(theta1 - si[i])
                #print "case 2"
            elif (si[i] >= (theta1+theta2)/2) and (si[i] < theta2) :
                F[i] = k1*(si[i] - theta2)
                #print "case 3"
            else:
                F[i] = k2*(np.tanh(rho*(si[i] - theta2))/rho)
                #print "case 4"
            for j in range(0,n_pre): 
                H[i][j] = (F[i]*sj[j])
        
        delta = np.dot(eta , H) + np.dot(eps ,np.subtract (weights_init , self.cij))
        self.cij += delta
        cij_trans = np.multiply(self.cij,transform)
        output = np.transpose(np.mat(cij_trans) * np.transpose(np.mat(sj)))
        self.output = output
        return output