# Importing necessary modules
import math
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt


# Importing csv files and reading the data
true_odo = genfromtxt('true_odo.csv', delimiter=',')
true_pos_x, true_pos_y = true_odo[1:,0], true_odo[1:,1]


# font = {'family': 'serif',
#         'color':  'darkred',
#         'weight': 'normal',
#         'size': 16,
#         }


# plt.figure(4, figsize=(12,8))
# plt.xlim(-1,6)
# plt.ylim(-1,6)
# plt.title('Robot Position in 2-D Env : From True /Odo Topic', fontweight='bold', fontdict=font, size=20, weight='bold')
# plt.xlabel('X Axis', fontdict=font, weight='bold')
# plt.ylabel('Y Axis', fontdict=font, weight='bold')

# plt.plot(true_pos_x,true_pos_y, linewidth=4, label='True Path')

# plt.scatter(0, 0, s=500, marker="^",c='orange')
# plt.scatter(5, 5, s=500, marker="*", c='orange')
# plt.scatter(1, 0, s=5000, marker="o", c='red')
# plt.scatter(0, 2, s=5000, marker="o", c='red')

# plt.text(1, 2.8, "Start Position", family="sans-serif", fontsize=14, fontdict=font)
# plt.arrow(1.2, 2.6, -1.1, -2.1, head_width=0.1, head_length=0.1, fc="grey", ec="grey")

# plt.text(0, 4.96, "Goal Location",family="sans-serif", fontsize=14, fontdict=font)
# plt.arrow(1.2, 5, 3.3, 0, head_width=0.1, head_length=0.1, fc="grey", ec="grey")

# plt.text(2, 1.4, "Obstacles",family="sans-serif", fontsize=16,fontdict=font)
# plt.arrow(1.9, 1.4, -0.6, -0.8, head_width=0.1, head_length=0.1, fc="grey", ec="grey")
# plt.arrow(1.9, 1.4, -1.4, 0.4, head_width=0.1, head_length=0.1, fc="grey", ec="grey")

# plt.legend(loc='best')
# plt.show()




# ************************************
#  ANIMATION : To save as a .mp4 file
# ************************************

from matplotlib.animation import FuncAnimation

fig = plt.figure()
ax = plt.axes(xlim=(-1, 6), ylim=(-1, 6))
line, = ax.plot([], [], lw=3)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    x = true_pos_x[:i+1]
    y = true_pos_y[:i+1]
    plt.scatter(1, 0, s=3000, marker="o", c='red')
    plt.scatter(0, 2, s=3000, marker="o", c='red')
    line.set_data(x, y)
    return line,

anim = FuncAnimation(fig, animate, init_func=init,
                               frames=len(true_pos_x), interval=10, blit=True)

anim.save('True-Path_60fps.mp4', fps=60)

