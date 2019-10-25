#!/usr/bin/python3

import csv
import math
import numpy as np
import matplotlib.pyplot as plt

'''
np_data = None
with open("angle_data.csv", 'r') as f:
    reader = csv.reader(f)
    first_row = True
    for row in reader:
        if first_row:
            np_data = np.asarray([float(r) for r in row]).reshape(1, 4)
            first_row = False
        else:
            np_data = np.concatenate((np_data, np.asarray([float(r) for r in row]).reshape(1 ,4)))

print(np_data.shape)
plt.plot(np_data[:,0], np_data[:,1], label='roll')
plt.plot(np_data[:,0], np_data[:,2], label='pitch')
plt.plot(np_data[:,0], np_data[:,3], label='yaw')
plt.legend()
plt.savefig('plot.png', dpi=300)
'''

m_0 = 1.2938

# for Jzz //yaw
T_0 = 2.505
g = 9.8
d = 0.12954
L = 1.8542
Jzz = m_0*g*(d**2)*(T_0**2)/(16*(math.pi**2)*L)
print('Jzz:', Jzz)

# for Jyy  //roll
T_0 = 3.486
g = 9.8
d = 0.12954
L = 1.75768
Jyy = m_0*g*(d**2)*(T_0**2)/(16*(math.pi**2)*L)
print('Jyy:', Jyy)

# for Jxx //pitch
T_0 = 3.882
g = 9.8
d = 0.07493
L = 1.78308
Jxx = m_0*g*(d**2)*(T_0**2)/(16*(math.pi**2)*L)
print('Jxx:', Jxx)



# calculate for Jzz

