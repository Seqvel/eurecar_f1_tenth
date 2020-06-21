import numpy as np
import math
from scipy import interpolate
from matplotlib import pyplot as plt

def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

def calc_arc_length(x, y):
    indices = len(x)-1
    arc_length_list = []
    for i in range(indices):
        d = calc_dist(x[i], y[i], x[i+1], y[i+1])
        arc_length_list.append(d)

    return np.array(arc_length_list)


x = np.array([23, 24, 24, 25, 25, 26, 28, 30])
y = np.array([13, 10, 13, 12, 13, 14, 15, 15.5])

# append the starting x,y coordinates
x = np.r_[x, x[0]]
y = np.r_[y, y[0]]

"""
Fit splines to x=f(u) and y=g(u), treating both as periodic.
    - s=0 is needed in order to force the spline fit to pass through all the input points.
    - tck: spline representation
    - u  : sample point normalized arc length (0 ~ 1)

"""
tck, u = interpolate.splprep([x, y], s=0, per=True)
# evaluate the spline fits for 1000 evenly spaced distance values
xi, yi = interpolate.splev(np.linspace(0, 1, 100), tck)

num_points = 500+1 # n
u_del = np.ones(num_points-1) / float(num_points-1) # n-1
u = np.array([sum(u_del[:i]) for i in range(len(u_del))]) # n-1 length. start from 0.
u = np.r_[u, [1.]] # n length. end to 1.0

xii, yii = interpolate.splev(u, tck)
arc_length_list = calc_arc_length(xii, yii)
arc_length_mean = arc_length_list.mean()
print("u :", u)
print("arc_length_list. mean:", arc_length_mean)

STEP_SIZE = 1. / len(u)

for _ in range(50):
    arc_length_error_list = arc_length_list - arc_length_mean # n-1
    u_del -= STEP_SIZE * arc_length_error_list # n-1
    u[:-1] = np.array([sum(u_del[:i]) for i in range(len(u_del))]) # update from ind 0 to ind -2.
    xii, yii = interpolate.splev(u, tck)
    arc_length_list = calc_arc_length(xii, yii)

print("u :", u)
print("arc_length_error_list mean :", arc_length_error_list.mean())
print("arc_length_list :")
print(arc_length_list)

# plot the result
fig, ax = plt.subplots(1, 1)
ax.plot(x, y, 'or')
ax.plot(xi, yi, '-b')
ax.plot(xii, yii, '.g')
plt.show()