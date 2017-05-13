'''
A simple function to simulate one node model of water heater
'''

import numpy as np
from .controller import thermostat_controller

# we define the simulator
def oneNodeSimulator(duration,               # [h]
                     W_t,                    # [gal/h]
                     F_t,                    # [Hz]
                     # Tunable Paras
                     T_0=100.,               # [F]
                     delta_t=1./3600,        # [h]
                     T_amb=65.,              # [F]
                     T_inlet=90.,            # [F]
                     T_s=120.,               # [F]
                     deadband=2.,            # [F]
                     m_0=0,                  # 
                     # Static Paras
                     r=1.,                   # [ft]
                     h=4.,                   # [ft]
                     R=10,                   # [h ft^2 F/BTU]
                     Cp=1,                   # [BTU/F lb]
                     P_r=10236.426,          # [BTU/h]
                     enable_control=False,
                     f_low=59.5,             # [Hz]
                     f_high=60.5):           # [Hz]
    # calculate tank properties
    SA = 2 * np.pi * r * (r + h)             # [ft^2]    
    V_tank = np.pi * r**2 * h * 7.48         # [gal]
    
    # static paras
    G = SA / R                               # [BTU/h F]
    C = 8.3 * V_tank * Cp                    # [BTU/F]
    
    # total number of iterations
    count = int(duration / delta_t)          
    
    T_sim = np.zeros(count+1)
    P_t = np.zeros(count)
    Ms = np.zeros(count)
    alphas = np.zeros(count)
    
    T_sim[0] = T_0

    T_t = T_0
    m_t = m_0
    for i in range(count):
        B_t = 8.3 * W_t[i] * Cp # convert from 1gal water to lb
        alpha_t = np.exp(-delta_t * (G + B_t) / C)
        f_t = F_t[i]
        m_t = thermostat_controller(m_t, T_t, T_s, deadband,
                                    reverse_ON_OFF=False,  
                                    enable_freq_control=enable_control,
                                    f_t=f_t, 
                                    f_low=f_low, 
                                    f_high=f_high)
        # track the status change    
        Ms[i] = m_t
        alphas[i] = alpha_t
        T_t = alpha_t * T_t + (1 - alpha_t) * (G / (G + B_t) * T_amb + 
                                               B_t / (G + B_t) * T_inlet + 
                                               m_t * P_r / (G + B_t))
        T_sim[i+1] = T_t
        P_t[i] = 0.293 * m_t * P_r

    return T_sim[:count], P_t, Ms, alphas
