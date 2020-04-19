# Importing necessary modules
import math
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt


# Importing csv files and reading the data
true_odo = genfromtxt('omega_time.csv', delimiter=',')
true_pos_x, true_pos_omega = true_odo[1:,0], true_odo[1:,1]
time = np.arange(len(true_pos_x))


font = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 16,
        }


plt.figure(4, figsize=(12,8))
plt.title('Angular Velocity vs Time', fontdict=font, fontweight='bold', size=20)

plt.xlim((-50,400))

# plt.plot(time, true_pos_x)
plt.plot(time,true_pos_omega, linewidth=4, label='Angular Change Over Time')

plt.xlabel('Time 'r'$ (sec) $', fontdict=font, weight='bold')
plt.ylabel('Angular Velocity  'r'$ (\omega) $', fontdict=font, weight='bold')


plt.annotate('Initial Rotation', xy=(-35, 0.2),  xycoords='data',
            xytext=(0, 0), textcoords='offset points',
            bbox=dict(boxstyle="round4,pad=.6", fc="0.9"), fontsize='x-large')

plt.annotate('Obstacle Avoidance', xy=(125, 0.08),  xycoords='data',
            xytext=(0, 0), textcoords='offset points',
            bbox=dict(boxstyle="round4,pad=.5", fc="0.9"), fontsize='xx-large' )

plt.annotate('Superficial adjustments as \n robot is very close to target', xy=(266, -0.1),  xycoords='data',
            xytext=(0, 0), textcoords='offset points',
            bbox=dict(boxstyle="round4,pad=.5", fc="0.9"), fontsize='large')

            

plt.legend(loc='best')
plt.show()



# ************************************
#  ANIMATION : To save as a .mp4 file
# ************************************

# from matplotlib.animation import FuncAnimation

# fig = plt.figure()
# ax = plt.axes(xlim=(0, 420), ylim=(-0.5, 0.75))
# line, = ax.plot([], [], lw=3)

# def init():
#     line.set_data([], [])
#     return line,

# def animate(i):
#     x = time[:i+1]
#     y = true_pos_omega[:i+1]
#     line.set_data(x, y)
#     return line,

# anim = FuncAnimation(fig, animate, init_func=init,
#                                frames=len(true_pos_omega), interval=10, blit=True)


# anim.save('Omega.mp4', fps=60)