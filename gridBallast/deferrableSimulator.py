'''
A simple function to simulate a deferrable load
'''

import numpy as np
import pandas as pd
from .controller import gridBallast_controller


def deferrableSimulator(T_cycle,
                        T_max,
                        step,
                        P_cycle,
                        F_t,
                        f_low,
                        f_high):
    # make sure P_cycle is in 1D dimension
    if len(P_cycle.shape) != 1 or len(P_cycle)!=int(T_cycle/step):
        print('Incorrect dimension of P_cycle! Use 1D numpy array.')
        return
    
    n_max = int(T_max / step)
    n_cycle = int(T_cycle / step)
    newP = np.zeros(n_max)
    oldP = np.zeros(n_max)
    oldP[:n_cycle] = P_cycle
    
    i = 0
    j = 0
    while i < n_max and j < n_cycle and (n_max-i)>(n_cycle-j):
        m_t = gridBallast_controller(F_t[i], 1, f_low, f_high)
        if m_t == 0:
            # it has to be OFF, delay the load
            i += 1
        else:
            # keep the load running
            newP[i] = P_cycle[j]
            j += 1
            i += 1
    # check if n_max-i == n_cycle-j, we fill in the rest of newP
    if (n_max-i) == (n_cycle-j):
        newP[-(n_max-i):] = P_cycle[-(n_cycle-j):]
    return oldP,newP
