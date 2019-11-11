#!/usr/bin/python3

import csv
import math
import numpy as np
import matplotlib.pyplot as plt

np_data = None
with open("for_traj.csv", 'r') as f:
    reader = csv.reader(f)
    first_row = True
    for row in reader:
        if first_row:
            np_data = np.asarray([float(r) for r in row]).reshape(1, 4)
            first_row = False
        else:
            np_data = np.concatenate((np_data, np.asarray([float(r) for r in row]).reshape(1 ,4)))

data_len= np_data.shape[0]
print('data_len:', data_len)
for i in range(0, data_len, 5):
    x = np_data[i, 0]
    y = np_data[i, 1]
    psi = np_data[i, 2]
    yaw = np_data[i, 3]
    arrow_width = 0.1
    arrow_len = 0.05
    plt.arrow(x, y, arrow_len*math.cos(psi), arrow_len*math.sin(psi),
            shape='left',
            head_width=arrow_width,
            head_length=arrow_width,
            head_starts_at_zero=True,
            length_includes_head=True,
            ec=None,
            fc='b')
    plt.arrow(x, y, arrow_len*math.cos(yaw), arrow_len*math.sin(yaw),
            shape='right',
            head_width=arrow_width,
            head_length=arrow_width,
            head_starts_at_zero=True,
            length_includes_head=True,
            ec=None,
            fc='r')

plt.ylim(-2, 1)
plt.xlim(-1, 2)
plt.xlabel('x axis (m)', fontsize=16)
plt.ylabel('y axis (m)', fontsize=16)
plt.title('Trajectory of following a square route', fontsize=16)
plt.plot(np_data[:,0], np_data[:,1], label='odometry')

plt.savefig('traj.png', dpi=300)

