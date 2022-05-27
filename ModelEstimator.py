import numpy as np
import control

class ModelEstimator:
    def __init__(self, Ts):
        print(Ts)
        self.epsilon = np.array([[0],[0],[0]])
        self.epsilondot = np.array([[0],[0],[0]])
        self.states = np.concatenate([self.epsilondot, self.epsilon])
        self.next_states = np.array([[0],[0],[0],[0],[0],[0]])
        self.Ts = Ts
        
        Kx = 1.22
        Ky = 1.013
        Kz = 1.176
        tau_x = 0.3
        tau_y = 0.16
        tau_z = 0.16
        
        A = np.matrix([[-1/tau_x,    0,         0,       0, 0, 0],
                       [ 0,         -1/tau_y,   0,       0, 0, 0],
                       [ 0,          0,        -1/tau_z, 0, 0, 0],
                       [ 1,          0,         0,       0, 0, 0],
                       [ 0,          1,         0,       0, 0, 0],
                       [ 0,          0,         1,       0, 0, 0]])

        B = np.matrix([[-Kx/tau_x,     0,      0],
                       [0,         -Ky/tau_y,  0],
                       [0,         0,         -Kz/tau_z],
                       [0,         0,         0],
                       [0,         0,         0],
                       [0,         0,         0]])

        C = np.matrix([[0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 0, 1]])

        D = np.matrix([[0,0,0],
                       [0,0,0],
                       [0,0,0]])
        
        H = control.ss(A,B,C,D)
        Hd = control.c2d(H, self.Ts, 'tustin')
        self.A, self.B, self.C = Hd.A, Hd.B, Hd.C
        
        
    def update(self, epsilon, epsilondot, u):
        #self.epsilon = epsilon
        #self.epsilondot = epsilondot
        next_epsilon = epsilondot * self.Ts + epsilon
        self.states = np.concatenate([next_epsilon, epsilon])
        #self.next_states = self.states#(self.A @ self.states) + (self.B @ u)
        self.next_states = (self.A @ self.states) + (self.B @ u)
        
    def estimate(self, u):
        # u - numpyarray of velocity/input to model
        self.states = self.next_states
        self.next_states = (self.A @ self.states) + (self.B @ u)
        return self.C @ self.states
        
        
        
        
        # d_states_prev = self.d_states 
        # self.d_states = (self.A @ self.states) + (self.B @ u)
        # self.states = ((d_states_prev + self.d_states)/2)*Ts
        # self.epsilon = self.C @ self.states
        # return self.epsilon