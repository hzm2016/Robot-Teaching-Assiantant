import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import numpy as np

""" ================================= Plot result ===================================== """
COLORS = ['blue', 'green', 'red', 'cyan', 'magenta', 'yellow', 'black', 'purple', 'pink',
          'brown', 'orange', 'teal', 'coral', 'lightblue', 'lime', 'lavender', 'turquoise',
          'darkgreen', 'tan', 'salmon', 'gold', 'lightpurple', 'darkred', 'darkblue']
""" ================================================================================= """

FONT_SIZE = 14
linewidth = 4
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = FONT_SIZE

root_path = './motor_control/bin/data/'
angle_list = np.loadtxt(root_path + 'angle_list.txt', skiprows=1)
torque_list = np.loadtxt(root_path + 'torque_list.txt', skiprows=1)
angle_vel_list = np.loadtxt(root_path + 'angle_vel_list.txt', skiprows=1)
angle_list_e = np.loadtxt('./motor_control/bin/2_font_3_angle_list.txt', delimiter=',', skiprows=1)

angle_list_1 = angle_list[:, 0]
angle_list_2 = angle_list[:, 1]

torque_list_1 = torque_list[:, 0]
torque_list_2 = torque_list[:, 1]

angle_vel_list_1 = angle_vel_list[:, 0]
angle_vel_list_2 = angle_vel_list[:, 1]

angle_list_1_e = angle_list_e[:, 0]
angle_list_2_e = angle_list_e[:, 1]

fig = plt.figure(figsize=(20, 8))
plt.subplot(2, 4, 1)
plt.subplots_adjust(wspace=2, hspace=0)

plt.plot(angle_list_1, linewidth=linewidth)
# plt.xlim([0, 128])
# plt.ylim([0, 128])
plt.xlabel('time($t$)')
plt.ylabel('$q_1$(rad)')
# plt.axis('equal')
plt.tight_layout()

plt.subplot(2, 4, 2)
plt.subplots_adjust(wspace=0.2, hspace=0.2)

plt.plot(angle_list_2, linewidth=linewidth)

# plt.xlim([0., 0.6])
# plt.ylim([0., 0.6])
plt.xlabel('time($t$)')
plt.ylabel('$q_2$(rad)')

plt.subplot(2, 4, 3)
plt.plot(torque_list_1, linewidth=linewidth-2)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_1$(Nm)')
plt.legend()

plt.subplot(2, 4, 4)
plt.plot(torque_list_2, linewidth=linewidth-2)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_2$(Nm)')
plt.legend()

plt.subplot(2, 4, 5)
plt.plot(angle_vel_list_1, linewidth=linewidth)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_1$(Nm)')
plt.legend()

plt.subplot(2, 4, 6)
plt.plot(angle_vel_list_2, linewidth=linewidth)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_2$(Nm)')
plt.legend()

plt.subplot(2, 4, 7)
plt.plot(angle_list_1_e, linewidth=linewidth)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_1$(Nm)')
plt.legend()

plt.subplot(2, 4, 8)
plt.plot(angle_list_2_e, linewidth=linewidth)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_2$(Nm)')
plt.legend()

plt.show()