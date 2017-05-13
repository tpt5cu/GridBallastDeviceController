'''
We define different controllers inside this file.
'''

# the GridBallast Controller to return ON/OFF status bases on frequency
# the controller will only affect the m_t, the idea is that, 
# given the frequency information,
# decide wheather to bring up the load or shut down the load
# the controller can also be called directly to override the thermostat_controller
# by letting enable_lock and force_OFF being True
def gridBallast_controller(f_t, m_t, f_low, f_high,
                           enable_lock=False, 
                           force_OFF=False):
    if enable_lock:
        if force_OFF:
            return 0
        else:
            # if force_OFF not true, forcing to be on
            return 1
    if f_t < f_low:
        # OFF if f_t too low
        m_t = 0
    elif f_t > f_high:
        # ON if f_t too high
        m_t = 1
    # if f_t is within the frequency deadband, we do nothing
    return m_t

# get ON/OFF state given current tempertuare and frequency
# thermostat has a higher priority
def thermostat_controller(m_t, T_t, T_s, deadband,
                          enable_freq_control=False,
                          f_t=60, 
                          f_low=59.9, 
                          f_high=60.1,
                          reverse_ON_OFF=False):
    if T_t < T_s - deadband/2.:
        # turn on the load
        m_t = 0 if reverse_ON_OFF else 1
    elif T_t > T_s + deadband/2.:
        # turn off the load
        m_t = 1 if reverse_ON_OFF else 0
    elif enable_freq_control:
        # we are in the safe zone of making m_t either 0 or 1 
        # without going out of thermal band
        # impose the control functionality only in this situation 
        # as it won't compromise the users' comfort level
        m_t = gridBallast_controller(f_t, m_t, f_low, f_high)
    return m_t
