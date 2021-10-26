from numpy.core.defchararray import title
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.fft import fft, fftfreq
from scipy.interpolate import interp1d
from scipy import signal
import pickle

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

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


    # plt.legend(labels)    

    if save:
        plt.savefig(filename+".png")

    if show:
        plt.show()
    return

def chirp(t, A, w0, wf, tf, x0):
    k = (wf - w0) / tf
    w = 0.5 * k*t + w0
    return A * np.sin((0.5 * k * t**2 + w0) * 2*np.pi) + x0

def filtering_data(x, t, t_ideal):
    x_filt = signal.filtfilt(b, a, x, method="gust")
    f_x_int = interp1d(t, x_filt, fill_value="extrapolate")
    x_final = f_x_int(t_ideal)
    return x_final

def data_process(data, t_ideal, N):
    t = data[:,1]
    I = data[:,4] * calib_data["motor_K"]
    F = data[:,8] * calib_data["force_B"]
    theta = data[:,6]

    F_filt = filtering_data(F, t, t_ideal)
    I_filt = filtering_data(I, t, t_ideal)

    F_fft = np.abs(fft(F_filt))[1:N//2]
    I_fft = np.abs(fft(I_filt))[1:N//2]
    TF = F_fft/I_fft

    theta_avg = np.rad2deg(np.mean(theta))
    return F_fft, I_fft, TF, theta_avg, I_filt, F_filt, t_ideal

def plot_results(F_fft, I_fft, TF, theta, freq, filename, I_filt = [], F_filt = [], t_ideal = []):
    default_path = "experiment_results/Experiment_2/FFT_results/"
    msg = "RJ angle is "+str(theta) + " [deg]"
    print(msg)
    TF_filt = signal.filtfilt(b, a, TF, method="gust")

    # Проверка по времени график
    if len(I_filt)!=0:
        plt.figure(figsize=[10, 10])
        plt.subplot(2,1,1)
        plt.plot(t_ideal, F_filt, "r", lw=1)
        plot_design(y_label="Handle force [N]", xlim = [0,30], show=False)

        plt.subplot(2,1,2)
        plt.plot(t_ideal, I_filt, "r", lw=1)
        plot_design(y_label="Motor torque [Nm]", x_label="Time [sec]", xlim = [0,30], save=True, filename=default_path+"p0_"+filename)

    # Первый график

    plt.figure(figsize=[10, 10])
    plt.subplot(2,1,1)
    plt.plot(freq, F_fft, "r", lw=1)
    plot_design(y_label="Handle force amplitude", xlim = [0,10], show=False)

    plt.subplot(2,1,2)
    plt.plot(freq, I_fft, "r", lw=1)
    plot_design(y_label="Motor torque amplitude", x_label="Frequency [Hz]", xlim = [0,10], save=True, filename=default_path+"p1_"+filename)

    # Второй график

    plt.figure(figsize=[10, 5])
    plt.plot(freq, TF, color = 'red', alpha = 0.2, linewidth = 1)
    plt.plot(freq, TF_filt, color = "red")
    plot_design(y_label="Transfer function", x_label="Frequency [Hz]", plot_title=msg, xlim = [0,10], ylim = [0, 1000], save=True, filename=default_path+"p2_"+filename)

    # В логарифмическом виде

    plt.figure(figsize=[10, 5])
    plt.xlim([0.1, 10])
    plt.semilogx(freq, 20.0*np.log10(abs(TF)), color = 'red', alpha = 0.2, linewidth = 1)
    plt.semilogx(freq, 20.0*np.log10(abs(TF_filt)), color = 'red', linewidth = 2.5)
    plot_design(y_label="Magnitude [dB]", x_label="Frequency [Hz]", plot_title=msg, save=True, filename=default_path+"p3_"+filename)

    return

def processing_one_angle(angle_i, freq, t_ideal, N, plot_exp = False, plot_angle = False):
    F_fft_gen = np.zeros(freq.shape)
    I_fft_gen = np.zeros(freq.shape)
    TF_gen = np.zeros(freq.shape)
    
    I_gen = np.zeros(t_ideal.shape)
    F_gen = np.zeros(t_ideal.shape)

    theta_gen = 0

    for i in [1,2,3]:
        filename = "experiment_results/Experiment_2/Chirp_angle_"+str(angle_i)+"_exp_"+str(i)+"_A_100.csv"
        data = pd.read_csv(filename)
        data = data.to_numpy()
        F_fft, I_fft, TF, theta, I_filt, F_filt, t_ideal = data_process(data, t_ideal, N)
        if plot_exp:
            plot_results(F_fft, I_fft, TF, theta, freq, "angle_"+str(angle_i)+"_exp_"+str(i))

        F_fft_gen += F_fft
        I_fft_gen += I_fft
        TF_gen += TF
        theta_gen += theta
        I_gen += I_filt
        F_gen += F_filt

    F_fft = F_fft_gen/3
    I_fft = I_fft_gen/3 
    TF = TF_gen/3 
    F = F_gen/3
    I = I_gen/3
    theta = round(theta_gen/3,2)

    TF_filt = signal.filtfilt(b, a, TF, method="gust")
    if plot_angle:
        plot_results(F_fft, I_fft, TF, theta, freq, "angle_"+str(angle_i), I_filt=I, F_filt=F, t_ideal=t_ideal)
    return TF_filt, theta

global calib_data
calib_data = load_obj("calib_data")

angle_i = 1
i = 1
filename = "experiment_results/Experiment_2/Chirp_angle_"+str(angle_i)+"_exp_"+str(i)+"_A_100.csv"
data = pd.read_csv(filename)
labels = list(data.columns.values)
data = data.to_numpy()

N = 90000
tf = 120
t_ideal = np.linspace(0, tf, N)
freq = fftfreq(N, tf/N)[1:N//2]

filter_freq = 50
global b, a
b, a = signal.butter(4, filter_freq*(t_ideal[1]-t_ideal[0]))


labels = []

for angle_i in [10,11, 2,3,4,5,6,7,8]:
    TF_filt_i, theta = processing_one_angle(angle_i, freq, t_ideal, N)
    if angle_i == 10:
        TFs = TF_filt_i
    else:
        TFs = np.vstack((TFs, TF_filt_i))
    labels.append(r'$\phi = '+str(theta)+'^\circ$')



plt.figure(figsize=[10, 5])

plt.legend(bbox_to_anchor=(1, 1),
           bbox_transform=plt.gcf().transFigure)
plt.ylim([1.5, 3])
plt.xlim([0.01, 10])
for i in range(len(labels)):
    plt.semilogx(freq, np.log10(TFs[i,:]), label=labels[i]) #, label=labels[i]

# plt.semilogx(freq, np.log10(TFs.T)) 
# plt.legend(labels)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)
plot_design(y_label="Magnitude [dB]", x_label="Frequency [Hz]", labels = labels, save=True, filename="experiment_results/Experiment_2/FFT_results/final_result")


# TF_filt_i, theta = processing_one_angle(11, freq, t_ideal, N, plot_angle=True)


# filename = "experiment_results/Experiment_2/Chirp_angle_11_exp_1_A_100.csv"
# data = pd.read_csv(filename)
# data = data.to_numpy()

# t = data[:,1]
# I = data[:,4] * calib_data["motor_K"]
# F = data[:,8] * calib_data["force_B"]
# theta = data[:,6]
# I_des = data[:,11] * calib_data["motor_K"]

# theta_avg = np.rad2deg(np.mean(theta))

# TSA = {"L": 320/10**3,     # String length [m]
#        "r": 0.8/10**3,     # String radius [m]
#        "dtheta": 100,      # Motor nominal speed [rad/sec]
#        "tau": 0.18,        # Motor nominal torque [N.m]
#        }


# theta_motor = data[:,2]

# RJ = {"R": 32 * 10**(-3)}
# x = theta*RJ["R"]

# J = theta_motor*TSA["r"]**2/(TSA["L"]-x)

# F = I/J


# I = I_des
# F_filt = filtering_data(F, t, t_ideal)
# I_filt = filtering_data(I, t, t_ideal)

# F_fft = np.abs(fft(F_filt))[1:N//2]
# I_fft = np.abs(fft(I_filt))[1:N//2]
# TF = F_fft/I_fft

# theta_avg = np.rad2deg(np.mean(theta))

# plot_results(F_fft, I_fft, TF, theta_avg, freq, "angle_"+str(angle_i)+"_modeled")