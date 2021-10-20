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
    filter_freq = 50
    b, a = signal.butter(4, filter_freq*(t_ideal[1]-t_ideal[0]))
    x_filt = signal.filtfilt(b, a, x, method="gust")
    f_x_int = interp1d(t, x_filt, fill_value="extrapolate")
    x_final = f_x_int(t_ideal)
    return x_final

def data_process(data, t_ideal, N):
    t = data[:,1]
    x = data[:,2]
    dx = data[:,3]
    I = data[:,4]
    F = data[:,8]
    I_des = data[:,11]
    freq = data[:,12]

    I_des_filt = filtering_data(I_des, t, t_ideal)
    F_filt = filtering_data(F, t, t_ideal)
    I_filt = filtering_data(I, t, t_ideal)


    I_des_fft = np.abs(fft(I_des_filt))[1:N//2]
    F_fft = np.abs(fft(F_filt))[1:N//2]
    I_fft = np.abs(fft(I_filt))[1:N//2]
    TF = F_fft/I_fft
    return F_fft, I_fft, TF
    

i = 1
filename = "experiment_results/Experiment_2/Chirp_exp_"+str(i)+"_A_100.csv"

N = 90000
tf = 120
t_ideal = np.linspace(0, tf, N)
freq = fftfreq(N, tf/N)[1:N//2]

data = pd.read_csv(filename)
labels = list(data.columns.values)
data = data.to_numpy()

# t = data[:,1]
# x = data[:,2]
# dx = data[:,3]
# I = data[:,4]
# F = data[:,8]
# I_des = data[:,11]
# freq = data[:,12]

# I_des_filt = filtering_data(I_des, t_ideal)
# F_filt = filtering_data(F, t_ideal)
# I_filt = filtering_data(I, t_ideal)


# I_des_fft = np.abs(fft(I_des_filt))[1:N//2]
# F_fft = np.abs(fft(F_filt))[1:N//2]
# I_fft = np.abs(fft(I_filt))[1:N//2]

F_fft, I_fft, TF = data_process(data, t_ideal, N)

# Первый график

plt.subplot(2,1,1)
plt.plot(freq, F_fft, "r", lw=1)
plot_design(y_label="Amplitude for handle", xlim = [0,10], show=False)

plt.subplot(2,1,2)
plt.plot(freq, I_fft, "r", lw=1)
plot_design(y_label="Amplitude for real current", x_label="Frequency [Hz]", xlim = [0,10])

# Второй график
plt.plot(freq, TF)
plot_design(y_label="Amplitude for transfer function", x_label="Frequency [Hz]", xlim = [0,10], ylim=[0,0.003])