from re import X
from numpy.core.defchararray import index
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy import signal
from scipy.interpolate import interp1d
import pickle


def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

def plot_design(x_label = "", y_label = "", plot_title = "", labels = [], xlim = None, ylim = None, show = True):
    plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
    plt.grid(True)

    plt.xlabel(x_label, labelpad = 10, size=11)
    plt.ylabel(y_label, labelpad = 10, size=11)
    plt.title(plot_title, pad=17)

    if xlim:
        plt.xlim(xlim)

    if ylim:
        plt.ylim(ylim) 

    if show:
        plt.show()

    plt.tight_layout()
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

def TSA_jacobian(TSA, phi, theta):
    J =  theta*TSA["r"]**2/((TSA["L"]-phi*TSA["R"])*TSA["R"])
    # J = TSA["r"]
    return J

def sliding_window(x,y,n = 1000):
    x_new = np.zeros((len(x)-n,))
    y_new = np.zeros((len(x)-n,))

    for i in range(len(x_new)):
        x_new[i] = np.mean(x[i:i+n])
        y_new[i] = np.mean(y[i:i+n])
    return x_new, y_new

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

    plt.figure(figsize=[6, 3])
    font = {'size': 12,
        'family': 'serif'
        }
    plt.rc('text', usetex=True)
    plt.rc('font', **font)

    plt.plot(x_data, y_data,'o', color = "#DFDCDC")
    plt.plot(x_data, y_filt, color = "#3C3C3C", label = labels[0])
    plt.plot(x_des[1:], y_filt[0]+y_des[1:], "r--", lw = 2, label = labels[1])
    plt.legend()
    plot_design(x_label = x_label, y_label = y_label, xlim = x_lim, ylim = y_lim)
    return

def filtering_data(y_data, freq = 0.06666740741563795):
    b, a = signal.butter(4, freq)
    
    y_filt = signal.filtfilt(b, a, y_data, method="gust")
    return y_filt



global calib_data
calib_data = load_obj("calib_data")

A = 45
w = 0.1
data = pd.read_csv("experiment_results/Experiment_1/Experiment_final_long.csv")

labels = list(data.columns.values)

data = data.to_numpy()

TSA = {"L": 340 *10**(-3),     # String length [mm]
       "r": 0.71 *10**(-3),     # String radius [mm]
       "R": 32 *10**(-3)}

t = data[:,1]
theta = data[:,2]
dtheta = data[:,3]
I = data[:,4] * calib_data["motor_K"]
delta_x = data[:,5]
phi = data[:,6]
T = data[:,8] * calib_data["force_B"] * (75*10**(-3))

theta_des = data[:,9]
dtheta_des = data[:,10]

phi_ideal = np.linspace(np.min(phi), np.max(phi), 10000)
des_theta = TSA_inverse_kinematics(TSA, phi_ideal)

x_data = np.abs(theta)
y_data = phi

x_des = des_theta
y_des = phi_ideal

labels = [r"Experimental results",r"Analitical solution"]
y_label = r"Rotation joint angle $\varphi$ [rad]"
x_label = r"Motor angle $\theta$ [rad]"

TSA_drawings(x_data, y_data, x_des, y_des, x_label, y_label, labels, plot_num = 1, x_lim=[0,330])

idx = np.argwhere((np.abs(phi)>0.15))
idx = np.reshape(idx, (len(idx,)))

phi = phi[idx]
I = np.abs(I[idx])
T = T[idx]
t = t[idx]

idx = np.argsort(phi)
phi = phi[idx]
T = T[idx]
I = I[idx]

T_filt = filtering_data(T)
I_filt = filtering_data(I)

TF = T_filt/I_filt

phi_ideal = np.linspace(np.min(np.abs(phi)), np.max(np.abs(phi)), 1000)
theta_ideal = TSA_inverse_kinematics(TSA, phi_ideal)
J = TSA_jacobian(TSA, phi_ideal, theta_ideal)
TF_ideal = 1/J

TF_filt = filtering_data(TF, freq = 0.01)

labels = [r"Experimental results",r"Analitical solution"]
y_label = r"Transfer function $\|\frac{\tau_{o}}{u}\|$"
x_label = r"Rotation joint angle $\varphi$ [rad]"
x_lim = [0,3.14]
y_lim = []
y_lim = [0, 200]

n_off = 1000
phi_off = phi[-n_off]
theta_off = TSA_inverse_kinematics(TSA, phi_off)
J_off = TSA_jacobian(TSA, phi_off, theta_off)


plt.figure(figsize=[6, 3])
font = {'size': 12,
    'family': 'serif'
    }
plt.rc('text', usetex=True)
plt.rc('font', **font)

plt.plot(phi, TF,'o', color = "#DFDCDC")
plt.plot(phi, TF_filt, color = "#3C3C3C", label = labels[0])
plt.plot(phi_ideal, TF_ideal - (1/J_off-TF_filt[-n_off]), "r--", lw = 2, label = labels[1])
plt.legend()
plot_design(x_label = x_label, y_label = y_label, xlim = x_lim, ylim = y_lim)
