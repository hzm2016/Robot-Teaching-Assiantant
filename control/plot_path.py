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

FONT_SIZE = 20
linewidth = 5
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = FONT_SIZE

root_path = './motor_control/data/'

angle_list_1 = np.loadtxt(root_path + 'angle_list_1.txt')
angle_list_2 = np.loadtxt(root_path + 'angle_list_2.txt')

torque_list_1 = np.loadtxt(root_path + 'torque_list_1.txt')
torque_list_2 = np.loadtxt(root_path + 'torque_list_2.txt')

fig = plt.figure(figsize=(20, 4))
plt.subplot(1, 4, 1)
plt.subplots_adjust(wspace=2, hspace=0)

plt.plot(angle_list_1, linewidth=linewidth)
# plt.xlim([0, 128])
# plt.ylim([0, 128])
plt.xlabel('time($t$)')
plt.ylabel('$q_1$(rad)')
# plt.axis('equal')
plt.tight_layout()

plt.subplot(1, 4, 2)
plt.subplots_adjust(wspace=0.2, hspace=0.2)

plt.plot(angle_list_2, linewidth=linewidth)

# plt.xlim([0., 0.6])
# plt.ylim([0., 0.6])
plt.xlabel('time($t$)')
plt.ylabel('$q_2$(rad)')

plt.subplot(1, 4, 3)
plt.plot(torque_list_1, linewidth=linewidth)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_1$(Nm)')
plt.legend()

plt.subplot(1, 4, 4)
plt.plot(torque_list_2, linewidth=linewidth)

plt.xlabel('time($t$)')
plt.ylabel('$\tau_2$(Nm)')
plt.legend()

plt.show()