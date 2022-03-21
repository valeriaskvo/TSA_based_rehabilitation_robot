import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import ConvexHull

rng = np.random.default_rng()
points = rng.random((30, 2))   # 30 random points in 2-D
hull = ConvexHull(points)

x = points[hull.vertices,0]
y = points[hull.vertices,1]

t = np.arange(0, len(x))

labels  =[r'$\phi = 21^\circ$', r'$\phi = '+str(22)+'^\circ$']
plt.subplot(2,1,1)
plt.plot(t,x, label = labels[0])
plt.plot(t,y, label = labels[1])

plt.legend(bbox_to_anchor=(1, 1),
           bbox_transform=plt.gcf().transFigure)
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='upper left',
           ncol=2, mode="expand", borderaxespad=0.)
plt.title(r'W1 disk and central $\phi$ subtracted')


plt.subplot(2,1,2)
plt.plot(points[:,0], points[:,1], 'o')
plt.fill(x, y, color = "#DAD6D6")
plt.plot(x,y)
plt.plot(x[0],y[0], "bo")
plt.show()

# plt.figure(figsize=(8, 8))
# plt.axis('equal')
# plt.fill(x, y)
# plt.plot(x,y, color = "black")
# plt.show()