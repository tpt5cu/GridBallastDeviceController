'''
We define a simple TCL simulator here, which is similar to one-node
mode of a water heater.
'''

import numpy as np
from .controller import thermostat_controller

# we define the TCL simulator (fridge in this example)
def TCLsimulator(duration,               # [h]
                T_0=40.,                 # [F] initial temp
                delta_t=1./3600,         # [h] time step
                C=263.,                  # [BTU /F]
                R=.1,                    # [h F/BTU]
                T_amb=65.,               # [F]
                m_t=0,                   #
                P_r=2000*3.4121,         # [BTU/h]
                sigma=0,
                T_s=40.,                 # [F] set point
                deadband=2.,             # [F]
                enable_control=False,
                F_t=None,
                f_low=59.9,
                f_high=60.1):
    
    count = int(duration / delta_t)
    
    T_sim = np.zeros(count)
    Ms = np.zeros(count)
    P_t = np.zeros(count)
    
    T_sim[0] = T_0

    alpha = np.exp(-delta_t/(C*R))

    T_gain = R*P_r

    T_t = T_0
    m_t = 0
    
    for i in range(count-1):
        # the fridge has a different mechanics of ON/OFF , we need to reverse
        if enable_control and F_t is not None:
            m_t = thermostat_controller(m_t, T_t, T_s, deadband,
                                        enable_control,
                                        F_t[i],f_low,f_high,
                                        True)
        else:
            m_t = thermostat_controller(m_t, T_t, T_s, deadband,
                                       reverse_ON_OFF=True)
        # track the status change    
        Ms[i] = m_t
        P_t[i] = m_t * P_r / 3.4121
        epsilon_t = sigma * np.random.randn(1)
        T_t = alpha * T_t + (1-alpha)*(T_amb - m_t * T_gain) + epsilon_t
        T_sim[i+1] = T_t

    return T_sim, P_t, (Ms,alpha,T_gain)
