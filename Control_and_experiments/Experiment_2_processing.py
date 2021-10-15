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

def plot_state_and_force(filename, wall_detection = True):
    data = pd.read_csv(filename)
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
    I_des = data[:,11]
    plt.figure(figsize=[15, 10])
    plt.tight_layout()

    plt.subplot(3,1,1)
    plt.plot(t, x/(2*np.pi), color = 'red',lw = 3.)
    plot_design(plot_title = "Motor state", y_label = labels[2], show=False)

    plt.subplot(3,1,2)
    plt.plot(t, dx, color = 'red',lw = 3.)
    plot_design(y_label = labels[3],show=False)

    plt.subplot(3,1,3)
    plt.plot(t, I)
    if wall_detection:
        plot_design(x_label = labels[1], y_label = labels[4])
    else:
        plt.plot(t, I_des, color = 'black', lw = 3., ls = '--')
        plot_design(x_label = labels[1], y_label = labels[4],labels = ["Real data", "Desired data"])


    plt.figure(figsize=[10, 10])
    plt.tight_layout()

    plt.subplot(2,1,1)
    plt.plot(t, F, "r", lw=1)
    plot_design(y_label=labels[8], plot_title="Handle force versus time",show=False)

    plt.subplot(2,1,2)
    plt.plot(t, I)
    if wall_detection:
        plot_design(x_label = labels[1], y_label = labels[4],plot_title="Motor current versus time")
    else:
        plt.plot(t, I_des, color = 'black', lw = 3., ls = '--')
        plot_design(x_label = labels[1], y_label = labels[4],plot_title="Motor current versus time",labels = ["Real data", "Desired data"])
    return

I0 = 94
A = 80

filename_wall = "experiment_results/Experiment_2/Wall_detection"+"_I0_"+str(int(I0))+"_A_"+str(A)+".csv"
plot_state_and_force(filename_wall)


filename_chirp = "experiment_results/Experiment_2/Chirp_I0_"+str(int(I0))+"_A_"+str(A)+".csv"
plot_state_and_force(filename_chirp, wall_detection=False)
