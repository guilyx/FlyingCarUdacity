from numpy import loadtxt as nploadtxt
from numpy import std as npstd
import matplotlib.pyplot as plt

gps = nploadtxt('./data/GPS_error.txt', delimiter=',', dtype='Float64', skiprows=1)
ax = nploadtxt('./data/AX_error.txt', delimiter=',', dtype='Float64', skiprows=1)

gps_std = npstd(gps[:,1])
ax_std = npstd(ax[:,1])

print('GPS Standard Deviation : ', gps_std)
print('AX Standard Deviation : ', ax_std)


'''
GPS Standard Deviation :  0.6692746045424977
AX Standard Deviation :  0.49330582154595654
'''

data = [gps, ax]
sub_titles = ['GPS.X', 'AX.X']
fig, axes = plt.subplots(2, 1, figsize = [10, 10])
i = 0
for ax in axes.flat:
    _time = data[i][:,0]
    _data = data[i][:,1]
    ax.plot(_time, _data, 'r')
    ax.set_xlabel('Time t(s)')
    ax.set_ylabel('Error')
    ax.grid()
    ax.set_title(sub_titles[i])
    i += 1
plt.show()