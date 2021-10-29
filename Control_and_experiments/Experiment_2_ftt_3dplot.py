import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import scipy.interpolate
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata

# def func(x, y):
#     return x*(1-x)*np.cos(4*np.pi*x) * np.sin(4*np.pi*y**2)**2

# grid_x, grid_y = np.mgrid[0:1:100j, 0:1:200j]
# print(grid_x.shape, grid_y.shape)

# rng = np.random.default_rng()
# points = rng.random((1000, 2))
# values = func(points[:,0], points[:,1])
# print(points.shape, values.shape)

# grid_z = griddata(points, values, (grid_x, grid_y), method='nearest')
# print(grid_z.shape)

with open('FTT_process.npy', 'rb') as f:
    x = np.load(f)      # freq
    y = np.load(f)      # theta
    z = np.load(f)      # TF

xi = np.linspace(x.min(), x.max(), 100)
yi = np.linspace(y.min(), y.max(), 50)
xi, yi = np.meshgrid(xi, yi)

points = np.hstack((x,y))

z = np.reshape(z, (len(z),))
print(points.shape, z.shape)

zi = griddata(points, z, (xi, yi), method='nearest')
print(zi.shape)

fig = plt.figure()
ax = Axes3D(fig)
ax.plot_surface(xi,yi,zi)
plt.show()

# fig, ax = plt.subplots(nrows=2)
# ax.contour(xi, yi, zi, levels=14, linewidths=0.5, colors='k')
# cntr1 = ax.contourf(xi, yi, zi, levels=14, cmap="RdBu_r")

# npts = 200
# ngridx = 100
# ngridy = 200

# fig.colorbar(cntr1, ax=ax)
# ax.plot(x, y, 'ko', ms=3)
# ax.set(xlim=(-2, 2), ylim=(-2, 2))
# ax.set_title('grid and contour (%d points, %d grid points)' %
#               (npts, ngridx * ngridy))






# spline = sp.interpolate.Rbf(x, y, z, function='thin-plate')

# xi = np.linspace(x.min(), x.max(), 50)
# yi = np.linspace(y.min(), y.max(), 50)
# xi, yi = np.meshgrid(xi, yi)

# zi = spline(xi,yi)
# print(zi.shape)

# fig = plt.figure()
# ax = Axes3D(fig)
# ax.plot_surface(xi,yi,zi)
# plt.show()

