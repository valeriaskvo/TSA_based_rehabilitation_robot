from cProfile import label
from scipy import signal
from scipy.interpolate import interp1d
import sys
sys.path.append('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments')

import matplotlib.pyplot as plt
plt.style.use('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/rj_stand/paper_plots_style.mplstyle')

import numpy as np
import csv

def load_data_csv(path):
    with open(path+".csv", 'r') as f:
        reader = csv.reader(f, delimiter=',')
        headers = next(reader)
        data = np.array(list(reader)).astype(float)
    return headers, data

def interpolation(x, y):
    f = interp1d(x, y, fill_value="extrapolate")
    return f(x)

def filtering_data(x):
    b, a = signal.ellip(4, 0.01, 120, 0.125)
    y = signal.filtfilt(b, a, x, method = "gust")
    return y

def remove_zeros(x):
    idx_zero = np.where(x < 0.01)
    x[idx_zero] = 0.01
    return x

def sliding_window(x, n = 100):
    x_filt = np.zeros((len(x)-n,))
    for i in range(len(x_filt)):
        x_filt[i] = np.mean(x[i:i+n])
    return x_filt

def data_preparing(path):
    headers, data = load_data_csv(path)

    idxs = {"t": 1,
            "q": 2,
            "dq": 3,
            "x":4,
            "phi": 5,
            "I": 6,
            "tau": 7,
            "F_tsa": 8,
            "F_hand": 9}

    q = data[:,idxs["q"]]
    phi = data[:,idxs["phi"]]
    tau = data[:,idxs["tau"]]
    F_tsa = data[:,idxs["F_tsa"]]

    # return q, phi, filtering_data(tau), filtering_data(F_tsa)
    return q, phi, tau, F_tsa

def data_combining(q, phi, tau, F_tsa, q_i, phi_i, tau_i, F_tsa_i):
    q = np.hstack((q, q_i))
    phi = np.hstack((phi, phi_i))
    tau = np.hstack((tau, tau_i))
    F_tsa = np.hstack((F_tsa, F_tsa_i))
    idx = np.argsort(q)
    return q[idx], phi[idx], tau[idx], F_tsa[idx]

path = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/RJ_experiments/Results/"
experiment_name = "static_experiment_v_10_"
exp_n = ["1", "2", "3", "4", "5"]
n = exp_n[0]

q, phi, tau, F_tsa = data_preparing(path+experiment_name+n)

# for n in exp_n:
#     q_i, phi_i, tau_i, F_tsa_i = data_preparing(path+experiment_name+n)
#     if n == "1":
#         q, phi, tau, F_tsa = q_i, phi_i, tau_i, F_tsa_i
#     else:
#         q, phi, tau, F_tsa = data_combining(q, phi, tau, F_tsa, q_i, phi_i, tau_i, F_tsa_i)

TSA = {"L": 315 *10**(-3),     # String length [mm]
       "r": 0.72 *10**(-3),     # String radius [mm]
       "R": 31.5 *10**(-3)}


phi_calc = (TSA["L"] - np.sqrt(TSA["L"]**2 - q**2*TSA["r"]**2))/TSA["R"]

# plt.figure(figsize = (5, 2.5))
# plt.plot(q, np.rad2deg(phi))
# plt.plot(q, np.rad2deg(phi_real), "r--")
# plt.tight_layout()
# plt.show()

q = remove_zeros(q)
tau = remove_zeros(tau)


tau_j = F_tsa * TSA["R"]
TF_real = sliding_window(tau_j)/sliding_window(tau)
# TF_real = sliding_window(TF_real)
phi_real = sliding_window(phi)

# TF = (TSA["R"]*(TSA["L"]-phi_calc*TSA["R"]))/(q*TSA["r"]**2)
J = (q*TSA["r"]**2)/(TSA["R"]*(TSA["L"]-phi_calc*TSA["R"]))

dq = remove_zeros(np.diff(q))
J_phi = np.diff(phi)/(dq)
J_phi = sliding_window(J_phi)
phi_real = sliding_window(phi[1:])

# plt.plot(np.rad2deg(phi_real), TF_real)
# plt.plot(np.rad2deg(phi), TF)

plt.plot(np.rad2deg(phi_real), J_phi)
plt.plot(np.rad2deg(phi), J)
plt.xlim([1, 150])
# plt.ylim([0, 200])
plt.show()


# fig, (ax1, ax2) = plt.subplots(2, 1, sharex = True)
# ax1.plot(q, tau, "black")
# ax1.plot(q, tau_proc, "r--")

# ax2.plot(q, F_tsa/TSA["R"], "black")
# ax2.plot(q, F_tsa_proc, "r--")
# plt.show()
