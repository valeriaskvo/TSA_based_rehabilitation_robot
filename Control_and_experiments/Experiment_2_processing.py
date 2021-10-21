import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_design(x_label = "", y_label = "", plot_title = "", labels = [], xlim = None, ylim = None, show = True, save = False, filename = ""):
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

    if save:
        plt.savefig(filename+".png")

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


    xlim = [0, 120]
    
    dt = np.diff(t)
    print("Time for save", np.average(dt), "[sec]")
    print("Frequency", 1/np.average(dt), "[Hz]")

    plt.figure(figsize=[15, 10])
    plt.tight_layout()

    plt.subplot(3,1,1)
    plt.plot(t, x/(2*np.pi), color = 'red',lw = 3.)
    plot_design(plot_title = "Motor state", y_label = labels[2], show=False, xlim=xlim)

    plt.subplot(3,1,2)
    plt.plot(t, dx, color = 'red',lw = 3.)
    plot_design(y_label = labels[3],show=False, xlim=xlim)

    plt.subplot(3,1,3)
    plt.plot(t, I)
    if wall_detection:
        plot_design(x_label = labels[1], y_label = labels[4], save = True, filename = filename[:-4]+"_motor_state")
    else:
        I_min = np.zeros(t.shape)+np.min(I_des)
        I_max = np.zeros(t.shape)+np.max(I_des)

        plt.plot(t, I_min, color = 'black', lw = 3., ls = '--')
        plt.plot(t, I_max, color = 'black', lw = 3., ls = '--')
        # plt.plot(t, I_des, color = 'black', lw = 3., ls = '--')
        plot_design(x_label = labels[1], y_label = labels[4],labels = ["Real data", "Desired limitations"], save = True, filename = filename[:-4]+"_motor_state", xlim=xlim)#, xlim = [119, 120]

        


    plt.figure(figsize=[10, 10])
    plt.tight_layout()

    plt.subplot(2,1,1)
    plt.plot(t, F, "r", lw=1)
    plot_design(y_label=labels[8], plot_title="Handle force versus time",show=False)

    plt.subplot(2,1,2)
    plt.plot(t, I)
    if wall_detection:
        plot_design(x_label = labels[1], y_label = labels[4],plot_title="Motor current versus time", save = True, filename = filename[:-4]+"_current_force")
    else:
        I_min = np.zeros(t.shape)+np.min(I_des)
        I_max = np.zeros(t.shape)+np.max(I_des)

        plt.plot(t, I_min, color = 'black', lw = 3., ls = '--')
        plt.plot(t, I_max, color = 'black', lw = 3., ls = '--')
        plot_design(x_label = labels[1], y_label = labels[4],plot_title="Motor current versus time",labels = ["Real data", "Desired limitations"], save = True, filename = filename[:-4]+"_current_force")
    return

I0 = 90
A = 30
# parameters = "_exp_3"
parameters = "_motor_only"
experiment_name = "Chirp" + parameters
wall_detection_name = "Wall_detection" + parameters


# filename_wall = "experiment_results/Experiment_2/"+wall_detection_name+"_A_"+str(A)+".csv"
# plot_state_and_force(filename_wall)


# filename_chirp = "experiment_results/Experiment_2/"+experiment_name+"_A_"+str(A)+".csv"

filename_chirp = "experiment_results/Experiment_2/Chirp_angle_2_exp_3_A_100.csv"
plot_state_and_force(filename_chirp, wall_detection=False)
