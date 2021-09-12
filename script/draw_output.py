#!/usr/bin/env python
import matplotlib.pyplot as plt
with open("../build/output.txt", 'r') as out:
    lines = out.readlines()
    est_px = []
    est_py = []
    meas_px = []
    meas_py = []
    gt_px = []
    gt_py = []
for line in lines:
    words = line.split('\t')
    est_px.append(words[0])
    est_py.append(words[1])
    meas_px.append(words[4])
    meas_py.append(words[5])
    gt_px.append(words[6])
    gt_py.append(words[7])
plt.plot(est_px, est_py, 'ro', meas_px, meas_py, 'g^', gt_px, gt_py, 'bs')
plt.show()
