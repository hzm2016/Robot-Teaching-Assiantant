import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns
import numpy
""" ================================= Plot result ===================================== """
COLORS = ['blue', 'green', 'red', 'cyan', 'magenta', 'yellow', 'black', 'purple', 'pink',
          'brown', 'orange', 'teal', 'coral', 'lightblue', 'lime', 'lavender', 'turquoise',
          'darkgreen', 'tan', 'salmon', 'gold', 'lightpurple', 'darkred', 'darkblue']

""" ================================================================================= """

YLABEL_LIST = ["Action(mm/$\circ$)", "Force/Moment(N/Nm)", "Position(mm)/Orientation$(\circ)$", "Reward"]
XLABEL_LIST = ["Steps"]

FONT_SIZE = 20
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = FONT_SIZE

path_data = np.loadtxt(root_path + '/' + font_name + '/2_font_' + str(type) +'.txt')

fig = plt.figure(figsize=(20, 4))
plt.subplot(1, 4, 1)
plt.subplots_adjust(wspace=2, hspace=0)

plt.plot(path_data[:, 1], path_data[:, 0], marker='o', linewidth=linewidth)
plt.plot(x_list, y_list, linewidth=linewidth - 2)
plt.xlim([0, 128])
plt.ylim([0, 128])
plt.xlabel('$x_1$')
plt.ylabel('$x_2$')
# plt.axis('equal')
plt.tight_layout()

plt.subplot(1, 4, 2)
plt.subplots_adjust(wspace=0.2, hspace=0.2)

plt.plot(x_1_list, x_2_list, linewidth=linewidth + 2, color='r')

plt.xlim([0., 0.6])
plt.ylim([0., 0.6])
plt.xlabel('$x_1$(m)')
plt.ylabel('$x_2$(m)')

plt.subplot(1, 4, 3)
plt.plot(t_list[1:], angle_1_list_e, linewidth=linewidth, label='$q_1$')
plt.plot(t_list[1:], angle_2_list_e, linewidth=linewidth, label='$q_2$')
plt.xlabel('Time (s)')
plt.ylabel('One-loop Angle (rad)')
plt.legend()

plt.subplot(1, 4, 4)
plt.plot(t_list[1:], angle_1_list_e, linewidth=linewidth, label='$q_1$')
plt.plot(t_list[1:], angle_2_list_e, linewidth=linewidth, label='$q_2$')
plt.xlabel('Time (s)')
plt.ylabel('One-loop Angle (rad)')
plt.legend()

plt.show()