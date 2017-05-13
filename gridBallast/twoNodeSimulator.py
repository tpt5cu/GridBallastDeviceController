'''
A more sophisticated two-node model
'''

import numpy as np
import pandas as pd

from .controller import thermostat_controller
from .controller import gridBallast_controller

class waterheater:
    global DEBUG
    DEBUG = False
    # init with default values
    def __init__(self,
                # water demand & grid frequency
                W_t = None,
                F_t = None, 
                # static paras
                tank_height = 4.,                        # [ft]
                tank_radius = 1.,                        # [ft]
                tank_top_to_upperHeatingPos = .2,        # [ft]
                tank_top_to_lowerHeatingPos = 3.7,       # [ft]
                # tank_hmax = 
                # fixed paras
                power_coef = 1,
                power_rate = 10236.426,                  # [BTU/h] Q_elec
                rho = 61.82,                             # [lb/ft^3]
                R = 10.,                                 # [ft^2 F h/BTU]
                Cp = 1.,                                 # [BTU/F lb]
                tau = 120,                               # [h]      
                f_low = 59.5,                            # [Hz]
                f_high = 60.5,                           # [Hz]
                # temperature
                T_inlet = 90.,                           # [F]
                T_amb = 65.,                             # [F]
                T_setpoint = 120.,                       # [F]
                T_deadband = 2.,                         # [F]
                T_BOIL = 212.,                           # [F] boiling temperature
                # simulation paras
                T_0 = 100.,                              # [F]
                duration = 10.,                          # [h]
                delta_t = 1./3600                        # [h]
                 ):
        # static paras
        self.tank_height = tank_height                # [ft]
        self.tank_radius = tank_radius                # [ft]
        self.tank_t2upper = tank_top_to_upperHeatingPos   # [ft]
        self.tank_t2lower = tank_top_to_lowerHeatingPos   # [ft]
        # fixed paras
        self.power_coef = power_coef
        self.power_rate = power_rate                  # [BTU/h] Q_elec
        self.rho = rho                                # [lb/ft^3]
        self.R = R                                    # [ft^2 F h/BTU]
        self.Cp = Cp                                  # [BTU/F lb]
        self.tau = tau                                # [h]      
        self.f_low = f_low                            # [Hz]
        self.f_high = f_high                          # [Hz]
        
        # temperature
        self.T_inlet = T_inlet                        # [F]
        self.T_amb = T_amb                            # [F]
        self.T_setpoint = T_setpoint                  # [F]
        self.T_deadband = T_deadband                  # [F]
        self.T_BOIL = T_BOIL                          # [F] boiling temperature
        
        # simulation paras
        self.T_0 = T_0                                # [F]
        self.duration = duration                      # [h]
        self.delta_t = delta_t                        # [h]
        
        # water draw leads to the switching between one-/two- node model
        self.st = 0 # one-node: heating element initial off state
        self.s1 = 0  # two-node: lower heating element  off
        self.s2 = 0  # two-node: upper heating element off
        self.flag = 1 # 1: one-node  2: two-node
        
        # tank specs
        self.tank_SA = 2 * np.pi * self.tank_radius * \
                      (self.tank_radius + self.tank_height)   # [ft^2]
        self.tank_volume = np.pi * self.tank_radius**2 * \
                           self.tank_height                   # [ft^3]
        # calculated paras
        self.C = self.rho * self.tank_volume * self.Cp        # [BTU/F]
        self.G = self.tank_SA / self.R                        # [BTU/F h]
        # temperature setting
        self.Tw = self.T_0                                    # [F] outlet water temp       
        self.T_lower = self.Tw                                # [F]
        self.T_upper = self.Tw                                # [F]
        
        # sim paras
        self.n_iters = int(self.duration / self.delta_t)  
        # water demand [gal/h] and frequenzy info [Hz]
        self.water_demand  = np.array([5] * self.n_iters) if W_t is None else W_t      
        self.Ft = np.random.normal(59.9, 0.4, int(self.n_iters)) if F_t is None else F_t
        # initial status is full of hot water
        self.hotWaterSlug_height = self.tank_height              # [ft] initialize as one-node
        
        # simulation output
        self.T_t = np.zeros(self.n_iters)
        self.Tlower_t = np.zeros(self.n_iters)
        self.Tupper_t = np.zeros(self.n_iters)
        self.FLAG_t = np.zeros(self.n_iters)
        self.P_t = np.zeros(self.n_iters)
        self.h_t = np.zeros(self.n_iters)
            
    # decide one/two node model based on h, w and dh/dt
    def isTwoNode(self, h, w):
        return self.dhdt(h, w) < 0 and h!=0
        
    def isOneNode(self, h, w):
        return (h == self.tank_height and self.dhdt(h, w) >= 0 ) or \
                (h == 0 and self.dhdt(h,w) <=0 )

    # calculate the gradient of hot water slug movement,
    # given current height of hot water slug and water demand
    def dhdt(self, h, w):
#         if self.T_upper == self.T_lower:
#             return 0
        # 7.48 is to convert gallon to ft^3, 7.48 gal=1 ft^3
        a = (self.power_coef * self.power_rate + self.G * (self.T_amb - 0)) / \
            (self.C * (self.T_upper - 0)) * self.tank_height - \
            self.rho * w / 7.48 * self.Cp * self.tank_height / self.C
        b = -self.G / self.C
        return a + b * h
    
    def update(self, h, a, b):
        return (np.exp(b * self.delta_t) * (a + b * h) - a) / b
    
    # we can introduce control inside this function
    def getOnOffState(self, T, st, enable_control=False, f=60):
        return thermostat_controller(st, T, self.T_setpoint, self.T_deadband,
                                     enable_control,f,self.f_low,self.f_high)
        #if T < self.T_setpoint - self.T_deadband / 2.:
        #    st = 1
        #elif T > self.T_setpoint+ self.T_deadband / 2.:
        #    st = 0
        #elif enable_control:
        #    # we check the frequency constraint
        #    # we are in the safe zone of making st either 0 or 1 
        #    # without going out of thermal band
        #    st = gridBallast_controller(f, st, self.f_low, self.f_high)
        #return st

    def oneNodeSim(self, T, w, enable_control, f):      
        st = self.getOnOffState(T, self.st, enable_control, f) 
        a = self.power_coef * self.power_rate * st / (self.Cp * self.rho * self.tank_volume) + \
            self.tank_SA * self.T_amb / (self.Cp * self.rho * self.tank_volume * self.R) + \
            w * self.T_inlet / 7.48 / self.tank_volume
        b = - (self.tank_SA /  (self.Cp * self.rho *  self.tank_volume * self.R) + \
             w / 7.84  / self.tank_volume)    
        T_new = self.update(T, a, b)    
        if DEBUG:
            print('\t singer layer -- s:%d, T:%.2f, V:%.2f,a:%.4f, b:%.4f, T_new:%.2f' \
                     % (st, T, self.tank_volume, a, b, T_new))    
        self.st = st
        P_new = self.power_rate * st / 3.412  # convert BTU/h to W
        return T_new, P_new
    
    def twoNodeSim(self, h, w, enable_control, f):
        '''
        h is the height of hot water slug at the begining of the delta_t
        self.hotWaterSlugHeight is the new height of the hot water slug

        We need to consider the diferent situations during the two-node models:
        1) the lower water is so little that it doesn't reach the lower thermostat
        2) the lower water just passes the lower thermostat
        3) the upper water is so little that it doesn't cover the upper thermostat
        
        Additionally, during the water usage events, the lower temp will be affected by
        the mixing of the old T1 and the new incoming T_inlet.
        
        Keep all these in mind when impelementing the model.
        '''
        # for two node models, we extract values for lower and upper parts
        # lower water
        h_new = self.hotWaterSlug_height
        L_inlet = w/7.48 * self.delta_t
        L1 = np.pi * self.tank_radius**2 * (self.tank_height - h)
        # lower temp is the mixing of old cold water and the inlet
        T1 = (self.T_lower * L1 + self.T_inlet * L_inlet) / (L1+L_inlet)
        # new L1 at the end of the delta_t
        L1 = np.pi * self.tank_radius**2 * (self.tank_height - h_new+0.001)
        if L1<=0:
            print('...bug...')
            L1=0.1
            
        # upper_water
        L2 = np.pi * self.tank_radius**2 * (h_new+0.001)
        if L2<=0:
            print('...bug...')
            L2=0.1
        T2 = self.T_upper
        
        # then we decide ON/OFF status of each heating element
        if h_new > self.tank_t2lower:
            # only consider top hot water layer
            s2 = self.getOnOffState(T2, self.s2, enable_control, f)
            s1 = 0
        elif h_new <= self.tank_t2upper and h_new >= self.tank_t2lower:
            # hot and cold water layer is separated
            s2 = self.getOnOffState(T2, self.s2, enable_control, f)
            s1 = self.getOnOffState(T1, self.s1, enable_control, f)
        elif h_new < self.tank_t2lower:
            s1 = self.getOnOffState(T1, self.s1, enable_control, f)
            s2 = 0
            
        # s1 and s2 cannot both be 1, the upper hot layer has a higher priority
        if s1 == 1 and s2 == 1:
            s1 = 0
        
        # we look at the top hot layer - L2 
        SA2 = 2 * np.pi * self.tank_radius * (self.tank_radius + h)
        a = self.power_coef * self.power_rate * s2 / (self.Cp * self.rho * L2) + \
            SA2 * self.T_amb / (self.Cp * self.rho * L2 * self.R) + \
            w * self.T_inlet / 7.48 / L2
        b = - (SA2 /  (self.Cp * self.rho * L2 * self.R) + \
            w / 7.84 / L2)    
        T2_new = self.update(T2, a, b)
        # check if T2_new is boiling, given L2 is very small
        T2_new = self.T_BOIL if T2_new > self.T_BOIL else T2_new  

        if DEBUG:
            print('\t upper hot layer -- s2:%d, T2:%.2f, L2:%.2f,a:%.4f, b:%.4f, T2_new:%.2f' \
                 % (s2, T2, L2, a, b, T2_new))
            
        # then, we look at lower cold layer - L1 
        SA1 = 2 * np.pi * self.tank_radius * (self.tank_radius + self.tank_height - h)
        a = self.power_coef * self.power_rate * s1 / (self.Cp * self.rho*L1) + \
            SA1 * self.T_amb / (self.Cp * self.rho * L1 * self.R)
        b = - SA1/ (self.Cp * self.rho * L1 * self.R)
        T1_new = self.update(T1, a, b)
        # check if T1_new is boiling, given L1 is very small
        T1_new = self.T_BOIL if T1_new > self.T_BOIL else T1_new
        if DEBUG:
            print('\t lower cold layer -- s1:%d, T1:%.2f, L1:%.2f,a:%.4f, b:%.4f, T1_new:%.2f' \
                 % (s1, T1, L1, a, b, T1_new))
        # Notice T1_new and T2_new here is not the temperature measured at the thermostat,
        # Instead, they are estimated average temperature of hot/cold water layer
        if h_new > self.tank_t2lower:
            # top hot water layer covers both thermostats
            T_upper_measure = T2_new
            T_lower_measure = T2_new
        elif h_new <= self.tank_t2upper and h_new >= self.tank_t2lower:
            # hot and cold water layer is separated
            T_upper_measure = T2_new
            T_lower_measure = T1_new
        elif h_new < self.tank_t2lower:
            # bottom cld water layer covers both thermostats
            T_upper_measure = T1_new
            T_lower_measure = T1_new
        
        # assume in two-node model, the heating capacity is same as the total power rate
        # however, only one can be on at the same time
        P_new = self.power_rate * (s1 + s2) / 3.412
        # update s1/s2 status
        self.s1 = s1
        self.s2 = s2
        
        return T1_new, T2_new, P_new, T_lower_measure, T_upper_measure
    
    # start simulation
    def simulate(self, enable_control=False):
        # go through each time interval and do the simulation
        for i in range(self.n_iters):
            h = self.hotWaterSlug_height
            w = self.water_demand[i]
            f = self.Ft[i]
            T = self.Tw 
            if DEBUG:
                print('Iteration %d, flgs:%d, h:%.2f, w:%.2f, Tw:%.2f, T_lower:%.2f, T_upper:%.2f, dhdt:%.2f'\
                     %(i, self.flag, h, w, T, self.T_lower, self.T_upper, self.dhdt(h,w)))            
            # we calculate the new height of hotWaterSlug after delta_t
            # surface are of hot water slug divided by R
            G2 =  2 * np.pi * self.tank_radius * (self.tank_radius + h) / self.R 
            a = (self.power_coef * self.power_rate + G2 * (self.T_amb - self.T_inlet)) / \
                (self.C*(self.T_upper-self.T_inlet)) * self.tank_height - \
                self.rho * w / 7.48 * self.Cp * self.tank_height / self.C
            b = -G2 / self.C
            # we update the height of hotWaterSlug
            self.G = G2
            h_new = self.update(h, a, b)
            if DEBUG:
                print('  previous node:%d, a:%.2f, b:%.2f, h_old:%.2f, h_new:%.2f, dhdt:%.2f' \
                      % (self.flag, a, b, h, h_new, self.dhdt(h, w)))   
                
            # make sure the new h is within the range
            if h_new < 0:
                h_new = 0
            elif h_new > self.tank_height:
                h_new = self.tank_height
            
            # decide self.flag for the current time interval
            # if two-node, we check if we want to switch from two-node to one-node model
            if self.flag == 2 and self.isOneNode(h, w):
                # print('\t switch from 2-node to 1-node')
                self.flag = 1
                # self.Tw = self.T_upper
            # if one-node, we check if we want to swtich from one-node to two-node model
            if self.flag == 1 and self.isTwoNode(h, w):
                # print('\t switch from 1-node to 2-node')                
                self.flag = 2
#                 if h== self.tank_height:
#                     self.T_lower = 
                # self.T_lower = self.T_inlet
                
            # we update the height of hotWaterSLug
            self.hotWaterSlug_height = h_new
#             h = h_new
            
            # if one node
            if self.flag == 1:
                T_new, P_new = self.oneNodeSim(T,w,enable_control,f)
                # update the temperature
                self.Tw = T_new
                self.T_lower = T_new
                self.T_upper = T_new
                T_lower_measure = T_new
                T_upper_measure = T_new
            # if two node
            elif self.flag == 2:
                T1_new, T2_new, P_new,  T_lower_measure, T_upper_measure =\
                        self.twoNodeSim(h,w,enable_control,f)
                # update the temperature
                self.Tw = T2_new
                self.T_lower = T1_new
                self.T_upper = T2_new
            # store historical simulation output over time
            self.T_t[i] = self.Tw
            self.Tlower_t[i] = T_lower_measure
            self.Tupper_t[i] = T_upper_measure
            self.FLAG_t[i] = self.flag
            self.P_t[i] = P_new
            self.h_t[i] = h
            
# end                                                  

