#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

data  = np.loadtxt("densities.txt")

# only take one value per tap
best_densities = []

for i in range(0,50):
    best_density_in_range = -1.0
    for row in range(data.shape[0]):
        time = data[row,1]
        density = data[row, 0]
        if time > i and time <= i+1:
            best_density_in_range = max(best_density_in_range,density)

    best_densities.append(best_density_in_range)
        

print(len(best_densities))
plt.plot(range(1,len(best_densities)+1),best_densities,label='density')
plt.xscale('log')
plt.legend()
plt.ylabel('density')
plt.xlabel('taps')
# plt.savefig("figure.pdf")
plt.show()
