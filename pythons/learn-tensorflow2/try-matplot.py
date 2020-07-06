import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np

fig = plt.figure()
ax = fig.gca(projection = '3d')

# Make data.
X = np.arange(-2, 2, 0.2)
Y = np.arange(-2, 2, 0.2)
X, Y = np.meshgrid(X, Y)
# R = np.sqrt(X ** 2 + Y ** 2)
# Z = np.sin(R)
Z = np.multiply(X, Y)

# Plot the surface.
surf = ax.plot_surface(
    X, Y, Z, cmap = cm.coolwarm,
    linewidth = 0, antialiased = False,
)

# Customize the z axis.
ax.set_zlim(-1.01, 1.01)
ax.zaxis.set_major_locator(LinearLocator(10))
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink = 0.5, aspect = 5)

# Use `sudo apt install python3-tk` to let below plt.show() work.
plt.show()
