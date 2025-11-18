import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --------------------------------------
# Soft robot arm as serpent head
# --------------------------------------

num_segments = 5
arm_length = 1.0
s = np.linspace(0, arm_length, num_segments)

# --------------------------------------
# Curvature model for serpent-like shape
# --------------------------------------
# First 40% rises upward
# Next 60% bends downward like a serpent head

rise_fraction = 0.4
split_idx = int(num_segments * rise_fraction)

# Upward curvature (gently arcs up)
theta_up = 1.2 * s[:split_idx]

# Downward curvature (bend toward negative z)
theta_down = -2.0 * (s[split_idx:] - s[split_idx])

# Combine
theta_z = np.concatenate([theta_up, theta_down])

# Add some side-to-side snake sway (optional)
theta_xy = 0.4 * np.sin(6 * np.pi * s)

# --------------------------------------
# Integrate to get XYZ coordinates
# --------------------------------------

x = [0]
y = [0]
z = [0]
ds = arm_length / num_segments

for i in range(1, num_segments):
    # direction based on snake-like curvature
    dx = np.cos(theta_z[i]) * ds
    dy = np.sin(theta_z[i]) * ds
    dz = np.cos(theta_z[i]) * np.sin(theta_xy[i]) * ds
    
    x.append(x[-1] + dx)
    y.append(y[-1] + dy)
    z.append(z[-1] + dz)

x = np.array(x)
y = np.array(y)
z = np.array(z)

# --------------------------------------
# Plotting
# --------------------------------------

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

ax.plot3D(x, y, z, linewidth=3)
ax.scatter3D(x, y, z, s=10)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_box_aspect([1,1,1])
plt.title("3D Serpent-Head Soft Robot Arm")
plt.show()
