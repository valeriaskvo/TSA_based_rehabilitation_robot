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

path = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/Exoskeleton_experiments/experiment_results/Lever_Experiment_1/"
# filename = "Velocity_20_weight_1_25.csv"
filename = "Current_150_weight_1_25.csv"

headers, data = load_data(path+filename)
for i in range(len(headers)):
    print(i, headers[i])

R = 55 * 10**(-3)
D = 210 * 10**(-3)
L = 285 * 10**(-3)
r = 1 * 10**(-3)
l = 190 * 10**(-3)
m = 1.25
g = 9.8
# Length string before separator 285
# Full sting length 625

idx_t = 1
idx_theta = 2
idx_dtheta = 3
idx_tau = 5
idx_alpha = 6
idx_F_tsa = 8
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


X = np.sqrt(R**2 + D**2 - 2*R*D*np.sin(alpha))
zeros = np.zeros(alpha.shape)

sin_gamma = D*np.cos(alpha)/X

tau_lever = F_tsa*sin_gamma*R

L_tsa = L+np.max(X)
J = theta*r**2/np.sqrt(L_tsa**2+theta**2*r**2)
TR_theor = 1/J*sin_gamma*R

TR = tau_lever/tau_m



# # # Transmission ratio
# # plt.scatter(np.rad2deg(alpha), TR)
# # plt.scatter(np.rad2deg(alpha), TR_theor, color = 'red')
# # plot_design(x_label='Lever angle [deg]', y_label='Transfer function', ylim=[-10, 1000])

# # # Velocity

# TR_v = np.diff(alpha)/np.diff(theta)
# TR_v_theor = J/(sin_gamma*R)

# plt.scatter(np.rad2deg(alpha[1:]), TR_v)
# plt.scatter(np.rad2deg(alpha), TR_v_theor, color = 'red')
# plot_design(x_label='Lever angle [deg]', y_label='Velocity jacobian', ylim = [0, 0.025])



# # Correctnes of force sensor
# tau_tsa = F_tsa*sin_gamma*R
# tau_mass = m*g*l*np.sin(alpha)
# plt.scatter(np.rad2deg(alpha), tau_lever)
# plt.scatter(np.rad2deg(alpha), tau_tsa)
# plt.scatter(np.rad2deg(alpha), tau_mass)
# plot_design(x_label='Lever angle [deg]', y_label='Lever torque')


# plt.subplot(3,1,1)
# plt.scatter(alpha, theta)
# # plt.plot(alpha, q_des, color = "red")

# plot_design(x_label = headers[idx_alpha], y_label = headers[idx_theta], show = False)

# plt.subplot(3,1,2)
# plt.scatter(alpha, dtheta)
# # plt.plot(alpha, dq_des, color = "red")

# plot_design(x_label = headers[idx_alpha], y_label = headers[idx_dtheta], show = False)

# plt.subplot(3,1,3)
# plt.scatter(alpha, tau_m)
# plt.plot(alpha, I_des, color = "red")
# plot_design(x_label = headers[idx_alpha], y_label = headers[idx_tau])


# plt.subplot(3,1,1)
# plt.scatter(alpha, tau_m)
# plot_design(x_label = headers[idx_alpha], y_label = headers[idx_tau], show = False)

# plt.subplot(3,1,2)
# plt.scatter(alpha, F_lever)
# plot_design(x_label = headers[idx_alpha], y_label = headers[idx_F_lever], show = False)

# plt.subplot(3,1,3)
# plt.scatter(alpha, F_lever/tau_m)
# plot_design(x_label = headers[idx_alpha], y_label = "Trnsmission ratio F_lever/tau_m")
