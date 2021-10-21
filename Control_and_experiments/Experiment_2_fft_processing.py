from numpy.core.defchararray import title
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.fft import fft, fftfreq
from scipy.interpolate import interp1d
from scipy import signal

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
    I = data[:,4]
    F = data[:,8]
    theta = data[:,6]

    F_filt = filtering_data(F, t, t_ideal)
    I_filt = filtering_data(I, t, t_ideal)

    F_fft = np.abs(fft(F_filt))[1:N//2]
    I_fft = np.abs(fft(I_filt))[1:N//2]
    TF = F_fft/I_fft

    theta_avg = np.rad2deg(np.mean(theta))
    return F_fft, I_fft, TF, theta_avg

def plot_results(F_fft, I_fft, TF, theta, freq, filename):
    default_path = "experiment_results/Experiment_2/FFT_results/"
    msg = "RJ angle is "+str(theta) + " [deg]"
    print(msg)
    TF_filt = signal.filtfilt(b, a, TF, method="gust")

    # Первый график

    plt.figure(figsize=[10, 10])
    plt.subplot(2,1,1)
    plt.plot(freq, F_fft, "r", lw=1)
    plot_design(y_label="Amplitude for handle", xlim = [0,10], show=False)

    plt.subplot(2,1,2)
    plt.plot(freq, I_fft, "r", lw=1)
    plot_design(y_label="Amplitude for real current", x_label="Frequency [Hz]", xlim = [0,10], save=True, filename=default_path+"p1_"+filename)

    # Второй график

    plt.figure(figsize=[10, 5])
    plt.plot(freq, TF, color = 'red', alpha = 0.2, linewidth = 1)
    plt.plot(freq, TF_filt, color = "red")
    plot_design(y_label="Amplitude for transfer function", x_label="Frequency [Hz]", plot_title=msg, xlim = [0,10], ylim=[0,0.003], save=True, filename=default_path+"p2_"+filename)

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
    theta_gen = 0

    for i in [1,2,3]:
        filename = "experiment_results/Experiment_2/Chirp_angle_"+str(angle_i)+"_exp_"+str(i)+"_A_100.csv"
        data = pd.read_csv(filename)
        data = data.to_numpy()
        F_fft, I_fft, TF, theta = data_process(data, t_ideal, N)
        if plot_exp:
            plot_results(F_fft, I_fft, TF, theta, freq, "angle_"+str(angle_i)+"_exp_"+str(i))

        F_fft_gen += F_fft
        I_fft_gen += I_fft
        TF_gen += TF
        theta_gen += theta

    F_fft = F_fft_gen/3
    I_fft = I_fft_gen/3 
    TF = TF_gen/3 
    theta = round(theta_gen/3,2)

    TF_filt = signal.filtfilt(b, a, TF, method="gust")
    if plot_angle:
        plot_results(F_fft, I_fft, TF, theta, freq, "angle_"+str(angle_i))
    return TF_filt, theta


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

for angle_i in [10,2,3,4,5,6,7,8]:
    TF_filt_i, theta = processing_one_angle(angle_i, freq, t_ideal, N, plot_exp = False)
    if angle_i == 10:
        TFs = TF_filt_i
    else:
        TFs = np.vstack((TFs, TF_filt_i))
    labels.append("RJ angle is "+str(theta)+" [deg]")

plt.figure(figsize=[10, 5])
plt.ylim([-90, -20])
plt.xlim([0.1, 10])
plt.semilogx(freq, 20.0*np.log10(TFs.T))
plot_design(y_label="Magnitude [dB]", x_label="Frequency [Hz]", labels = labels, save=True, filename="experiment_results/Experiment_2/FFT_results/final_result")



