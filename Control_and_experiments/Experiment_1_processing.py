from re import X
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy.spatial import ConvexHull
from scipy import signal
from scipy.interpolate import interp1d

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

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

def TSA_inverse_kinematics(TSA, phi):
  theta = np.sqrt((TSA["L"]**2-(TSA["L"]-phi*TSA["R"])**2)/TSA["r"]**2)
  return theta

def TSA_jacobian(TSA, theta):
    J =  theta*TSA["r"]**2/(TSA["L"]**2-theta**2*TSA["r"]**2)**0.5/TSA["R"]
    return J

def TSA_drawings(x_data, y_data, x_des, y_des, x_label, y_label, labels, plot_num ,x_lim = [], y_lim = []):
    
    if plot_num == 1:
        idx = np.argsort(x_data)
        x_data = x_data[idx]
        y_data = y_data[idx] 


    points = np.vstack((x_data, y_data))
    points = points.T
    hull = ConvexHull(points)

    x_region = points[hull.vertices,0]
    y_region = points[hull.vertices,1]

    if plot_num ==1:
        b, a = signal.butter(4, 0.01)
    else:
        b, a = signal.butter(4, 0.001)
    
    y_filt = signal.filtfilt(b, a, y_data, method="gust")

    plt.figure(figsize=[10, 5])
    plt.plot(x_data, y_data, color = "#DFDCDC")
    plt.plot(x_data, y_filt, color = "#3C3C3C", lw = 2, label = labels[0])
    plt.plot(x_des, y_des, "r--", lw = 2, label = labels[1])
    plt.legend()
    plot_design(x_label = x_label, y_label = y_label, xlim = x_lim, ylim = y_lim)
    return


global calib_data
calib_data = load_obj("calib_data")

A = 45
w = 0.1
data = pd.read_csv("experiment_results/Experiment_1/Experiment_final_1.csv")

labels = list(data.columns.values)

data = data.to_numpy()

t = data[:,1]
theta = data[:,2]
dtheta = data[:,3]
I = data[:,4] * calib_data["motor_K"]
delta_x = data[:,5]
phi = data[:,6]
T = data[:,7] * calib_data["force_A"]
F = data[:,8] * calib_data["force_B"]

theta_des = data[:,9]
dtheta_des = data[:,10]

TSA = {"L": 320 *10**(-3),     # String length [mm]
       "r": 0.7 *10**(-3),     # String radius [mm]
       "R": 32 *10**(-3)}


phi_ideal = np.linspace(np.min(phi), np.max(phi), 10000)
des_theta = TSA_inverse_kinematics(TSA, phi_ideal)


x_data = np.abs(theta)
y_data = phi

x_des = des_theta
y_des = phi_ideal

labels = [r"Experimental results",r"Analitical solution"]
y_label = r"Rotation joint angle $\phi$ [rad]"
x_label = r"Motor angle $\theta$ [rad]"

TSA_drawings(x_data, y_data, x_des, y_des, x_label, y_label, labels, plot_num = 1, x_lim=[0,350])


J = TSA_jacobian(TSA, theta_des[:-1])

x_data = dtheta[:-1]
y_data = np.diff(phi)/np.diff(t)

x_des = dtheta_des[:-1]
y_des = dtheta_des[:-1]*J

labels = [r"Experimental results",r"Analitical solution"]
y_label = r"Rotation joint velocity $\dot{\phi}$ [rad/sec]"
x_label = r"Motor speed $\dot{\theta}$ [rad/sec]"

TSA_drawings(x_data, y_data, x_des, y_des, x_label, y_label, labels, plot_num = 2)