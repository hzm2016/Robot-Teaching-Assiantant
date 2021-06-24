import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import seaborn as sns

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

