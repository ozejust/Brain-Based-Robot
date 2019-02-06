import numpy as np

class synapse_vd:
    def __init__(self,theta1=0.1, theta2=0.25
             , k1=0.45, k2=0.45, eta=0.125, eps=0.00125, rho=6.0
             , w_rand_min=0.02,  w_rand_max=0.03,n_pre=196,n_post=784,transform=0.):
                 
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
        self.f = [0.1,0.1,0.3,0.7,1.0,1.0,0.7,0.3,0.1]
        self.d = [  1,  2,  3,  4,  5,  6,  7,  8,  9]
        self.V = 0.
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
        self.f =np.roll(self.f,1)
        self.d =np.roll(self.d,1)
        f = self.f
        d = self.d
        sj=x[0:n_pre]
        si=x[n_pre:n_post+n_pre]
        #S = x[0:4:-1]
        S = x[n_post+n_pre:n_post+n_pre+4]        
        S_bar = np.average(S)
        V = self.V
        V = 1+f[1]*(S_bar+V*(d[0]-1))/d[1]
        self.V = V
        
        #F = []        
        F = np.zeros(n_post)
        H = np.zeros((n_post,n_pre))
        

        for i in range(0,n_post):

            if (si[i] < theta1):
                F[i] = 0.0
                #F.append(0.0)
                #print "case 1"
            elif (si[i] >= theta1 and si[i] < (theta1+theta2)/2) :
                F[i] = k1*(theta1 - si[i])
                #F.append(k1*(theta1 - si[i]))
                #print "case 2"
            elif (si[i] >= (theta1+theta2)/2) and (si[i] < theta2) :
                F[i] = k1*(si[i] - theta2)
                #F.append(k1*(si[i] - theta2))
                #print "case 3"
            else:
                F[i] = k2*(np.tanh(rho*(si[i] - theta2)))/rho
                #F.append(k2*(np.tanh(rho*(si[i] - theta2))/rho))
                #print "case 4"
            for j in range(0,n_pre): 
                H[i][j] = (F[i]*sj[j])
        
        delta = np.dot(eta*V , H) + eps* np.subtract (weights_init , self.cij)
        self.cij += delta
        cij_trans = np.multiply(self.cij,transform)
        output = np.transpose(np.mat(cij_trans) * np.transpose(np.mat(sj)))
        self.output = output
        #print "delta= " , delta
        #print "weights_init= " , weights_init
        #print "Cij= " , self.cij
        #print "output= " , np.max(self.output)
        return output