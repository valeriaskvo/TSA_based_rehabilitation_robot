import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_design(x_label = "", y_label = "", plot_title = "", labels = [], xlim = None, ylim = None, show = True):
    plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
    plt.grid(True)

    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(plot_title, pad=17)

    if xlim:
        plt.xlim(xlim)

    if ylim:
        plt.ylim(ylim)


    plt.legend(labels)    

    if show:
        plt.show()
    return

def plot_motor_state(A, w, data, labels, t, x, dx, I):
    A = A *2*np.pi
    w = w *2*np.pi

    x_des = data[:,9]
    dx_des = data[:,10]

    print(x_des[-1]/(2*np.pi), x[-1]/(2*np.pi),(x_des[-1]-x[-1])/(2*np.pi))

    plt.figure(figsize=[15, 10])
    plt.tight_layout()

    plt.subplot(3,1,1)
    plt.plot(t, x/(2*np.pi), color = 'red')
    plt.plot(t, x_des/(2*np.pi), color = 'black', lw = 3., ls = '--')
    plot_design(plot_title = "Motor state", y_label = labels[2], labels = ["Real data", "Desired data"], show=False)

    plt.subplot(3,1,2)
    plt.plot(t, dx, color = 'red')
    plt.plot(t, dx_des, color = 'black', lw = 3., ls = '--')
    plot_design(y_label = labels[3], labels = ["Real data", "Desired data"],show=False)

    plt.subplot(3,1,3)
    plt.plot(t, I)
    plot_design(x_label = labels[1], y_label = labels[4])
    return

def TSA_inverse_kinematics(TSA, delta_x):
  theta = np.sqrt((TSA["L"]**2-(TSA["L"]-delta_x)**2)/TSA["r"]**2)
  return theta

def TSA_jacobian(TSA, delta_x, x):
    J =  x*TSA["r"]**2/(TSA["L"]-delta_x)
    return J

A = 45
w = 0.1
data = pd.read_csv("experiment_results/Experiment_1/Experiment_final.csv")

labels = list(data.columns.values)

data = data.to_numpy()

t = data[:,1]
x = data[:,2]
dx = data[:,3]
I = data[:,4]
delta_x = data[:,5]
theta = data[:,6]
T = data[:,7]
F = data[:,8]

TSA = {"L": 320,     # String length [mm]
       "r": 0.8,     # String radius [mm]
       }

des_x = TSA_inverse_kinematics(TSA, delta_x)
J = TSA_jacobian(TSA, delta_x, x)
des_d_delta_x = J*dx
T_des = I*J

plt.figure(figsize=[10, 5])
plt.plot(t,np.rad2deg(theta),"r", lw = 3)
plot_design(x_label=labels[1], y_label = "Joint angle [deg]", plot_title = "Angle in rotation joint versus time")
print(np.max(np.rad2deg(theta)))

# plot_motor_state(A, w, data, labels, t, x, dx, I)

# # String contraction
# plt.figure(figsize=[10, 5])
# plt.plot(np.abs(x),delta_x, "r:", lw=1.5)
# plt.plot(des_x, delta_x, color = 'black', lw = 3.)
# plot_design(x_label=labels[2], y_label=labels[5], plot_title="String contraction versus motor angle", labels = ["Real data", "Desired data"])

# # String contraction velocity
# d_delta_x = np.diff(delta_x)/np.diff(t)

# plt.figure(figsize=[10, 5])
# plt.plot(x[:-1],d_delta_x, "r:", lw=1.5)
# plt.plot(x, des_d_delta_x, color = 'black', lw = 3.)
# plot_design(x_label=labels[2], y_label="String contraction velocity [mm/sec]", plot_title="Velocity of string contraction versus motor angle", labels = ["Real data", "Desired data"])

# # Tension

# plt.figure(figsize=[10, 10])
# plt.tight_layout()

# plt.subplot(2,1,1)
# plt.plot(x, T, "r", lw=1)
# plot_design(y_label=labels[7], plot_title="String tension versus motor angle", show = False)

# plt.subplot(2,1,2)
# plt.plot(x, T_des, color = 'r', lw = 1)
# plot_design(x_label=labels[2], y_label="Calculated string tension [units]")

# # # Tension from contraction
# # plt.figure(figsize=[10, 10])
# # plt.tight_layout()

# # plt.subplot(2,1,1)
# # plt.plot(delta_x, T, "r", lw=1)
# # plot_design(y_label=labels[7], plot_title="String tension versus string contraction", show = False)

# # plt.subplot(2,1,2)
# # plt.plot(delta_x, T_des, color = 'r', lw = 1)
# # plot_design(x_label=labels[5], y_label="Calculated string tension [units]")


# # plt.figure(figsize=[10, 5])
# # plt.plot(delta_x,I/F,"r", lw = 1.)
# # plot_design(x_label=labels[5], y_label = "Transmission ratio [units]")

# # plt.figure(figsize=[10, 5])
# # plt.plot(T,F,"r", lw = 1.)
# # plot_design(x_label=labels[7], y_label = labels[7], plot_title="Force on handle versus string tension")