import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import scipy.interpolate
from mpl_toolkits.mplot3d import Axes3D

with open('FTT_process.npy', 'rb') as f:
    x = np.load(f)      # freq
    y = np.load(f)      # theta
    z = np.load(f)      # TF

plt.plot(y)
plt.show()

# spline = sp.interpolate.Rbf(x,y,z,function='thin-plate')

# xi = np.linspace(x.min(), x.max(), 50)
# yi = np.linspace(y.min(), y.max(), 50)
# xi, yi = np.meshgrid(xi, yi)

# zi = spline(xi,yi)

# fig = plt.figure()
# ax = Axes3D(fig)
# ax.plot_surface(xi,yi,zi)
# plt.show()