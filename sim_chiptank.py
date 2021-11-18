"""

Simulator of wood-chip tank

Author: Finn Haugen, USN, finn.haugen@usn.no

Updated 2020 09 27

"""

# %% Import

import matplotlib.pyplot as plt
import numpy as np

# %% Time settings:

ts = 1  # Time-step [s]
t_start = 0.0  # [s]
t_stop = 2000.0  # [s]
N_sim = int((t_stop-t_start)/ts) + 1

# %% Process params:

rho = 145  # [kg/m^3]
A = 13.4  # [m^2]
t_delay = 250.0  # [s]
h_min = 0  # [m]
h_max = 15  # [m]
u_min = 0 # [kg/s]
u_max = 50 # [kg/s]

# %% Initialization of time delay:

u_delayed_init = 25  # [kg/s]
N_delay = int(round(t_delay/ts)) + 1
delay_array = np.zeros(N_delay) + u_delayed_init

# %% Arrays for plotting:

t_array = np.zeros(N_sim)
h_array = np.zeros(N_sim)
u_array = np.zeros(N_sim)
h_sp_array = np.zeros(N_sim)

# %% Initial state:

h_k = 10.0  # m setpoint
h_sp = h_k # just for graph
h_kp1 = 0.0 # m initializing only
k_pu = 26 # from labview simulation

# %% Simulation for-loop:

for k in range(0, N_sim):

    t_k = k*ts

    if t_k <= 250:
        u_man = 25  # kg/s
        w_out_k = 30  # kg/s
    else:
        u_man = 30  # kg/s
        w_out_k = 30  # kg/s
   
    # PID Tuning with the Ziegler-Nichols
    
    k_p = 0.45*k_pu
    e_k = h_sp - h_kp1 # control error
    u_p_k = k_p*e_k # p-term
    u_k = u_man + u_p_k # total control signal
    u_k = np.clip(u_k, u_min, u_max)
    
    # Time delay:
    u_delayed_k = delay_array[-1]
    delay_array[1:] = delay_array[0:-1]
    delay_array[0] = u_k

    # Euler-forward integration (Euler step):
    w_in_k = u_delayed_k  # kg/s
    dh_dt_k = (1/(rho*A))*(w_in_k - w_out_k)
    h_kp1 = h_k + ts*dh_dt_k
    h_kp1 = np.clip(h_kp1, h_min, h_max)

    # Storage for plotting:
    t_array[k] = t_k
    u_array[k] = u_k
    h_array[k] = h_k
    h_sp_array[k] = h_sp
    
    # Time shift:
    h_k = h_kp1

# %% Printing control error

print('The control error is: ', e_k)

# %% Plotting:

plt.close('all')
plt.figure(1)

plt.subplot(2, 1, 1)
plt.plot(t_array, h_array, 'b', label='h_level')
plt.plot(t_array, h_sp_array, 'g', label='h_setpoint')
plt.legend()
plt.grid()
plt.xlim(t_start, t_stop)
plt.xlabel('t [s]')
plt.ylabel('[m]')

plt.subplot(2, 1, 2)
plt.plot(t_array, u_array, 'g', label='control signal')
plt.legend()
plt.grid()
plt.xlim(t_start, t_stop)
plt.xlabel('t [s]')
plt.ylabel('[kg/s]')

plt.show()


