import numpy as np
import matplotlib.pyplot as plt
import csv

def plot_design(x_label = "", y_label = "", plot_title = "", labels = [], xlim = None, ylim = None, show = True):
    plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
    plt.grid(True)

    plt.xlabel(x_label, labelpad = 10, size=11)
    plt.ylabel(y_label, labelpad = 10, size=11)
    # plt.title(plot_title, pad=17)

    if xlim:
        plt.xlim(xlim)

    if ylim:
        plt.ylim(ylim) 

    if show:
        plt.show()

    plt.tight_layout()
    return

def load_data(path):
    with open(path, 'r') as f:
        reader = csv.reader(f, delimiter=',')
        headers = next(reader)
        data = np.array(list(reader)).astype(float)
    return headers, data

path = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/Exoskeleton_experiments/experiment_results/Lever_Stand_experiments/"
filename = "Current_300_weight_0.csv"
# filename = "Current_150_weight_1_25.csv"


headers, data = load_data(path+filename)
for i in range(len(headers)):
    print(i, headers[i])

R = 55 * 10**(-3)
D = 430 * 10**(-3)
L_tsa = 380 * 10**(-3)
L_f = 50 * 10**(-3)
r = 1 * 10**(-3)

m = 1.25
g = 9.8
L_m = 225 * 10**(-3)
# Length string before separator 285
# Full sting length 625

idx_t = 1
idx_theta = 2
idx_dtheta = 3
idx_tau = 5
idx_alpha = 6
idx_F_tsa = 7
idx_F_lever = 10

t = data[:,idx_t]
theta = data[:,idx_theta]
dtheta = data[:,idx_dtheta]
tau_m = data[:,idx_tau]
alpha = data[:,idx_alpha]

F_tsa = data[:,idx_F_tsa]
F_lever = data[:,idx_F_lever]

q_des = data[:,11]
dq_des = data[:,12]
I_des = data[:,14]

plt.subplot(3,1,1)
plt.plot(np.rad2deg(alpha),theta/(2*np.pi) )
plt.ylabel('Theta motor')

plt.subplot(3,1,2)
plt.plot(np.rad2deg(alpha),dtheta)
plt.ylabel('Motor angular velocity')

plt.subplot(3,1,3)
plt.plot(np.rad2deg(alpha),F_tsa/9.8)
plt.ylabel('Force sensor')
plt.show()

# n = 100
# alpha_calib = []
# dq_calib = []
# df_calib = []

# for i in range(len(dtheta)-n):
#     alpha_calib.append(np.mean(alpha[i:i+n]))
#     dq_calib.append(np.mean(dtheta[i:i+n]))
#     df_calib.append(np.mean(F_tsa[i:i+n]))

# plt.subplot(2,1,1)
# plt.plot(np.rad2deg(alpha_calib),dq_calib)
# plt.ylabel('Theta motor')

# plt.subplot(2,1,2)
# plt.plot(np.rad2deg(alpha_calib),df_calib)
# plt.ylabel('Force sensor')
# plt.show()