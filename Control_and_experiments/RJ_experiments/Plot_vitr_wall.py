import sys
sys.path.append('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments')

import matplotlib.pyplot as plt
plt.style.use('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/rj_stand/paper_plots_style.mplstyle')

import numpy as np
import csv
from rj_stand import load_calib
from scipy.interpolate import interp1d
from scipy.spatial import ConvexHull
import matplotlib as mpl

def load_data_csv(path):
    with open(path+".csv", 'r') as f:
        reader = csv.reader(f, delimiter=',')
        headers = next(reader)
        data = np.array(list(reader)).astype(float)
    return headers, data

def sliding_window(x, n = 100):
    x_filt = np.zeros((len(x)-n,))
    for i in range(len(x_filt)):
        x_filt[i] = np.mean(x[i:i+n])
    return x_filt

def nearest_neighborhood_filter(x, y, max_d):
    x_filt, y_filt = [], []
    
    while len(x) > 0:
        x_0, y_0 = x[0], y[0]
        d = np.sqrt((x-x_0)**2 + (y-y_0)**2)
        idx = np.argwhere(d < max_d)
        x_filt.append(np.mean(x[idx]))
        y_filt.append(np.mean(y[idx]))

        x, y = np.delete(x, idx), np.delete(y, idx)

    x_filt, y_filt = np.array(x_filt), np.array(y_filt)
    points = np.vstack((x_filt, y_filt)).T
    hull = ConvexHull(points)

    x_hull, y_hull = points[hull.vertices,0], points[hull.vertices,1]
    
    return np.hstack((x_hull, x_hull[0])), np.hstack((y_hull, y_hull[0]))

def data_processing(name):
    headers, data = load_data_csv(name)

    idxs = {"t": 1,
            "q": 2,
            "dq": 3,
            "x":4,
            "phi": 5,
            "I": 6,
            "tau": 7,
            "F_tsa": 8,
            "F_hand": 9}

    t = data[:,idxs["t"]]
    phi = data[:,idxs["phi"]]
    tau = data[:,idxs["tau"]]
    F_hand = data[:,idxs["F_hand"]]

    tau_hand = F_hand * calib_data["l"]

    max_tau_d = 0.01
    phi_tau, tau_f = nearest_neighborhood_filter(phi, tau, max_tau_d)

    max_tau_hand = 1
    phi_tau_hand, tau_hand_f = nearest_neighborhood_filter(phi, tau_hand, max_tau_hand)
    
    return phi_tau, tau_f, phi_tau_hand, tau_hand_f *10**(-3)

calib_data = load_calib()

path = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/RJ_experiments/Results/"
experiment_name = "wirtual_wall_k_"


colors = ['red', 'green', 'indigo', 'seagreen', 'saddlebrown']
data = {"1000_1": None,
     "1500_1": None,
     "2000_1": None}

colormap = {"1000_1": [r"$K_w$ = 1 $\frac{Nm}{rad}$", 'indigo', '-',0.3],
            "1500_1": [r"$K_w$ = 1.5 $\frac{Nm}{rad}$", 'green', '--', 0.6],
            "2000_1": [r"$K_w$ = 2 $\frac{Nm}{rad}$", 'red', '-.', 1]}

for key in data:
    phi_tau, tau_f, phi_tau_hand, tau_hand_f = data_processing(path+experiment_name+key)
    data[key] = phi_tau, tau_f, phi_tau_hand, tau_hand_f

# mpl.rc({'figure.subplot.left': 0.18,
#         'figure.subplot.right' :  0.92,
#         'figure.subplot.bottom' : 0.2,
#         'figure.subplot.top' :    0.87})

fig, ax = plt.subplots(figsize=(5, 3))

for key in data:
    phi = data[key][0]
    tau = data[key][1]
    ax.plot(np.rad2deg(phi), tau, color=colormap[key][1],
                                  label = colormap[key][0])

ax.set_ylabel(ylabel=r'Motor torque $u$ [Nm]')
ax.set_xlabel(xlabel=r'Joint angle $\varphi$ [deg]')

ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1),
          bbox_transform=fig.transFigure , ncol=3)
plt.savefig(path+"Figure_1.pdf", dpi=300)
plt.show()


# fig, ax = plt.subplots(figsize=(5, 3))

# for key in data:
#     phi = data[key][2]
#     tau = data[key][3]
#     ax.plot(np.rad2deg(phi), tau, color=colormap[key][1],
#                                   label = colormap[key][0])

# ax.set_ylabel(ylabel=r'Handle torque $\tau_o$ [Nm]')
# ax.set_xlabel(xlabel=r'Joint angle $\varphi$ [deg]')

# ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1),
#           bbox_transform=fig.transFigure , ncol=3)

# plt.savefig(path+"Figure_2.pdf", dpi=300)
# plt.show()

# mpl.rc({'figure.subplot.left': 0.1,
#         'figure.subplot.right' :  0.83,
#         'figure.subplot.bottom' : 0.18,
#         'figure.subplot.top' :    0.87})

# fig, ax = plt.subplots(figsize=(5, 3))
# ax_x = ax.twinx()

# for key in data:
#     phi = data[key][0]
#     tau = data[key][1]
    
#     phi_hand = data[key][2]
#     tau_hand = data[key][3]

#     ax.plot(np.rad2deg(phi_hand), tau_hand, color = 'blue',
#                                             alpha = colormap[key][3],
#                                             linestyle = colormap[key][2],
#                                             label = colormap[key][0])
#     ax_x.plot(np.rad2deg(phi), tau, linewidth=2, color = 'red',
#                                                  alpha = colormap[key][3],
#                                                  linestyle = colormap[key][2],
#                                                  label = colormap[key][0])


# ax.set_ylabel(ylabel=r'Handle torque $\tau_o$ [Nm]', color = 'tab:blue')
# ax_x.set_ylabel(r'Motor torque $u$ [Nm]', color = 'tab:red')
# ax.set_xlabel(xlabel=r'Joint angle $\varphi$ [deg]')

# ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1),
#           bbox_transform=fig.transFigure , ncol=3)
# plt.savefig(path+"Figure_3.pdf", dpi=300)

# plt.show()