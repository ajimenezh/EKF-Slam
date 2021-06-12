import math
import numpy as np
import matplotlib.pyplot as plt

def unit(v):
    return v / math.sqrt(v[0]*v[0] + v[1]*v[1])

r0 = 2.0
x = np.asarray([0.0, r0])
u = 0.5
v0 = r0 * math.sqrt(u / r0)
v = np.asarray([v0, 0.0])

xx = []
yy = []

delta_t = 0.01
for i in range(10000):

    a = - unit(x)*u

    v = v + a*delta_t
    
    x = x + v*delta_t

    xx.append(x[0])
    yy.append(x[1])

    u_t = unit(v)

plt.plot(xx, yy)
plt.show()
