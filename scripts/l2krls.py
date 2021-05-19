#!/usr/bin/python
#Filename:l2krls.py

import numpy as np

class L2KRLS:
    def __init__(self, output_num):
        print("in _init_")
        self.output_num = output_num
        self.alpha = 0.01
        self.mu = 0.05
        self.delta = 1.2
        self.ALD_buf = []
        self.Is_model_Initialed = False

    def init_model(self, x):
        print(x)
        if self.Is_model_Initialed == False:
            x = np.array(x)
            print(x)
            self.ALD_buf = [x]
            self.K_inv = np.matrix([1.0])
            self.Sn = np.matrix((1/(self.alpha+0.00001))*np.eye(1))
            self.Mn = np.matrix(np.zeros((1, self.output_num)))
            self.Is_model_Initialed = True
            print("model initialized success!!!") 
            
    def construct_Phi2(self, x, ALD_buf):
        ALD_arr = np.array(ALD_buf)
        x = np.matrix(x, dtype=np.float64).T
        X = np.matrix(ALD_arr, dtype=np.float64).T
        dX = x - X
        return np.exp(-np.sum(np.power(dX.T, 2), axis=1)/(2*self.delta*self.delta))   
    
    def Guassion_kernel(self, x, u):
        x = np.matrix(x,dtype=np.float64).T
        u = np.matrix(u,dtype=np.float64).T
        return np.exp(-(np.dot((x-u).T, (x-u)))/(2*self.delta*self.delta))    
        
    def update(self, x, y): 
        x = np.array(x)       
        y = np.matrix(y)
        k_t = self.construct_Phi2(x, self.ALD_buf)        
        k_tt = self.Guassion_kernel(x, x)
        a_t = np.dot(self.K_inv, k_t)
        delta_t = k_tt - np.dot(k_t.T, a_t)
        qn = np.dot(k_t,k_t.T)/(1+np.dot(np.dot(k_t.T, self.Sn), k_t))
        Gn = np.eye(len(self.Sn))-np.dot(self.Sn, qn)
        if abs(delta_t[0,0]) > self.mu:
            self.ALD_buf.append(x)
            self.K_inv = self.K_inv*delta_t[0,0]+np.dot(a_t, a_t.T)
            self.K_inv = np.row_stack((self.K_inv, -a_t.T))
            add_c = np.row_stack((-a_t, 1))
            self.K_inv = np.column_stack((self.K_inv, add_c))/delta_t[0,0]

            et = k_t-self.alpha*a_t
            ht = a_t+np.dot(self.Sn, et)
            gt = self.alpha+delta_t[0,0]-np.dot(ht.T,et)+np.dot(np.dot(ht.T,qn),ht)

            Sn_add2 = np.dot(Gn,ht)
            Sn_add1 = gt[0,0]*np.dot(Gn,self.Sn)+np.dot(Sn_add2,Sn_add2.T)
            Sn_add3 = np.row_stack((Sn_add1, -Sn_add2.T))
            Sn_add4 = np.row_stack((-Sn_add2, 1))
            Sn1 = np.column_stack((Sn_add3, Sn_add4))/gt[0,0]

            ft = self.Mn+np.dot(np.dot(self.Sn,k_t),y)
            pt = delta_t[0,0]*y-np.dot((np.dot(et.T,Gn)-np.dot(a_t.T,qn)),ft)

            Mn_add1 = gt[0,0]*np.dot(Gn,ft)-np.dot(Sn_add2,pt)
            Mn_add2 = pt
            Mn1 = np.row_stack((Mn_add1, Mn_add2))/gt[0,0]

            self.Sn = Sn1
            self.Mn = Mn1
        else:
            pre_y = self.compute_output(x)
            if (np.abs(pre_y[0] - y[0,0]) > 10.0):
                Sn1 = np.dot(Gn,self.Sn)
                Mn1 = np.dot(np.dot(Sn1, k_t),y)+np.dot(Gn, self.Mn)
                self.Sn = Sn1
                self.Mn = Mn1     
            
    def compute_output(self, x):
        x = np.array(x)
        k_t = self.construct_Phi2(x, self.ALD_buf)
        y = np.array(np.dot(k_t.T,self.Mn)).reshape(-1)
        return y 

    def print_weights(self):
        print("-----------network info-----------")
        print("network node number is:"+str(self.ALD_buf))    
        print("weights : ")
        print(self.Mn)
