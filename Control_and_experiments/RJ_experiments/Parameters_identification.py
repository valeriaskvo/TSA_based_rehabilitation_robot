import sys
sys.path.append('/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments')

import matplotlib.pyplot as plt
import numpy as np
import csv
from rj_stand import load_calib, save_calib
from rj_stand import Stand_RJ

def load_data_csv(path):
    with open(path+".csv", 'r') as f:
        reader = csv.reader(f, delimiter=',')
        headers = next(reader)
        data = np.array(list(reader)).astype(float)
    return headers, data

def plot_design(ax, legend=0):
  ax.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
  if legend==0:
    ax.legend()
  return

path = "/home/valeria/TSA_based_rehabilitation_robot/Control_and_experiments/RJ_experiments/Results/"
experiment_name = "static_experiment_v_const_"

headers, data = load_data_csv(path+"test")

for i in range(len(headers)):
    print(i, headers[i])

idxs = {"t": 1,
        "q": 2,
        "dq": 3,
        "x":4,
        "phi": 5,
        "I": 6,
        "tau": 7,
        "F_tsa": 8,
        "F_hand": 9}

# t = data[:,idxs["t"]]
# q = data[:,idxs["q"]]
# dq = data[:,idxs["dq"]]
# phi = data[:,idxs["phi"]]
# I = data[:,idxs["I"]]
# tau = data[:,idxs["tau"]]
# F_tsa = data[:,idxs["F_tsa"]]
# F_hand = data[:,idxs["F_hand"]]

# f, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10,15))

# for i in ["1", "2"]:
#     headers, data = load_data_csv(path+experiment_name+i)
#     t = data[:,idxs["t"]]
#     q = data[:,idxs["q"]]
#     dq = data[:,idxs["dq"]]
#     phi = data[:,idxs["phi"]]
#     I = data[:,idxs["I"]]
#     tau = data[:,idxs["tau"]]
#     F_tsa = data[:,idxs["F_tsa"]]
#     F_hand = data[:,idxs["F_hand"]]

    # ax1.plot(np.rad2deg(phi), q/(2*np.pi), linewidth=2)
    # ax2.plot(np.rad2deg(phi), dq, linewidth=2)
    # ax3.plot(np.rad2deg(phi), F_tsa, linewidth=2)

# ax1.set(ylabel="Motor angle [rev]", xlabel="Joint angle [deg]")
# plot_design(ax1,1)

# ax2.set(ylabel=headers[idxs["dq"]], xlabel="Joint angle [deg]")
# plot_design(ax2,1)

# ax3.set(ylabel=headers[idxs["F_tsa"]], xlabel="Joint angle [deg]")
# plot_design(ax3,1)
# plt.show()

def get_ident_data(i):
    _, data = load_data_csv(path+experiment_name+i)
    x = data[:,idxs["x"]]

    id = np.arange(len(x))
    id = id[x > 0.1]

    q = data[id,idxs["q"]]
    x = data[id,idxs["x"]]
    dx = np.diff(x)
    dq = np.diff(q)
    q = q[1:]
    x = x[1:]
    return q, x, dq, dx

def least_squares(q, x, dq, dx):
    L = 312.5
    
    a = np.reshape(q*dq, (len(q), 1))
    b = (L-x)*dx
    r_sqr = np.linalg.lstsq(a, b, rcond=None)[0][0]
    print("L:", L, "mm, r_sqr:",np.sqrt(r_sqr), "mm")
    return L, r_sqr

def correctnes_checking(q, x, dq, dx, L, r_sqr, name):
    J = q*r_sqr/(L-x)
    dx_check = J*dq

    plt.plot(x, dx, color="green")
    plt.plot(x, dx_check, color="red")
    plt.title(name)
    plt.show()
    return 

# q, x, dq, dx = get_ident_data("1")
# print("1 check")
# L, r_sqr = least_squares(q, x, dq, dx)
# correctnes_checking(q, x, dq, dx, L, r_sqr, "1 check")

# q_ = q
# x_ = x
# dx_ = dx
# dq_ = dq

# for i in ["2", "3"]:
#     q, x, dq, dx = get_ident_data(i)
#     print(i+"check")
#     L, r_sqr = least_squares(q, x, dq, dx)
#     correctnes_checking(q, x, dq, dx, L, r_sqr, i+" check")
#     q_ = np.hstack((q_,q))
#     x_ = np.hstack((x_,x))
#     dx_ = np.hstack((dx_,dx))
#     dq_ = np.hstack((dq_,dq))

# print("Summary check")
# L, r_sqr = least_squares(q_, x_, dq_, dx_)
# correctnes_checking(q_, x_, dq_, dx_, L, r_sqr, "Summary check")

# R = 31.5  # pulley radius
# l = 75   # handle l
# L = L     # string length
# r = np.sqrt(r_sqr)  # string radius
# # r = np.sqrt(r_sqr)

# calib_data = load_calib()
# calib_data["R"] = R
# calib_data["l"] = l
# calib_data["L"] = L
# calib_data["r"] = r


# stand_rj = Stand_RJ()
# stand_rj.get_state()

# while stand_rj.state["t"]<10:
#     stand_rj.get_state()

# bias_a = np.asarray(stand_rj.data[stand_rj.from_state_to_data["F_tsa"]])
# bias_b = np.asarray(stand_rj.data[stand_rj.from_state_to_data["F_hand"]])

# calib_data["bias_a"] = np.mean(bias_a)
# calib_data["bias_b"] = np.mean(bias_b)

# print(calib_data)

# save_calib(calib_data)


calib_data = load_calib()

x = data[:,idxs["x"]]
id = np.arange(len(x))
id = id[x > 1]

t = data[id,idxs["t"]]
q = data[id,idxs["q"]]
dq = data[id,idxs["dq"]]
x = data[id,idxs["x"]]
I = data[id,idxs["I"]]
tau = data[id,idxs["tau"]]
F_tsa = data[id,idxs["F_tsa"]]

J = q*calib_data["r"]**2*10**3/(calib_data["L"]-x) 
# F_tsa_check = J*tau
# plt.plot(x, F_tsa, color="green")
# plt.plot(x, F_tsa_check, color="red")
# plt.title("check Jacobian by tension force")
# plt.show()

# tau_check = F_tsa/J
# plt.plot(x, tau, color="green")
# plt.plot(x, tau_check, color="red")
# plt.title("check Jacobian by motor torque")
# plt.show()


dx = np.diff(x)
dq = np.diff(q)
q = q[1:]
x = x[1:]

J = q*calib_data["r"]**2/(calib_data["L"]-x)
dx_check = J*dq

plt.plot(x, dx, color="green")
plt.plot(x, dx_check, color="red")
plt.title("check Jacobian by linear velocity")
plt.show()

dq_check = dx/J
plt.plot(x, dq, color="green")
plt.plot(x, dq_check, color="red", alpha = 0.5)
plt.title("check Jacobian by motor velocity")
plt.show()

