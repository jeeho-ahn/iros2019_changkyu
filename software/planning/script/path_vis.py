import numpy as np
import matplotlib.pyplot as plt
import re

# ─── PASTE YOUR PATH HERE ────────────────────────────────────────────────────────
raw_path = '''
1 1 0 
1.47943 1.12242 0.5 
1.90605 1.38313 0.551175 
2.37858 1.52992 0.0511753 
2.85112 1.6767 0.551175 
3.21132 2.01991 0.851083 
3.61759 2.30385 0.445932 
3.99733 2.62108 0.945932 
4.17867 3.0815 1.42745 
4.36832 3.53852 0.927453 
4.7021 3.91062 0.83018 
4.93524 4.34707 1.33018 
5.06286 4.8305 1.31238 
5.06702 5.32529 1.81238 
5.07119 5.82008 1.31238 
5.31201 6.25235 0.823897 
5.54788 6.68732 1.3239 
5.78376 7.12229 0.823897 
6.19929 7.39092 0.323897 
6.19929 7.39092 0.323897 
6.67656 7.53996 0.302198 
7.15391 7.68877 0.302198 
7.63125 7.83758 0.302198 
8.10859 7.98639 0.302198 
8.5681 8.17704 0.604198 
8.5681 8.17704 0.604198 
8.8931 8.55015 1.1042 
9 9 1.5708 
'''
# ────────────────────────────────────────────────────────────────────────────────

# ─── PASTE YOUR OBSTACLES HERE ───────────────────────────────────────────────────
raw_obs = '''
  [0] Center=(3, 3), Yaw=0.785398 rad, Width=2, Height=1
  [1] Center=(7, 6), Yaw=0 rad, Width=1, Height=3
'''
# ────────────────────────────────────────────────────────────────────────────────

# Parse the path
lines = [l for l in raw_path.strip().splitlines() if l.strip()]
data = np.array([list(map(float, l.split())) for l in lines])
x, y, yaw = data[:,0], data[:,1], data[:,2]

# Parse the obstacles
obs = []
for line in raw_obs.strip().splitlines():
    m = re.search(
        r'Center=\(\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\),\s*'
        r'Yaw=([-\d\.]+)\s*rad,\s*'
        r'Width=([-\d\.]+)\s*,\s*Height=([-\d\.]+)',
        line
    )
    if not m: 
        continue
    cx, cy = float(m.group(1)), float(m.group(2))
    yaw_o   = float(m.group(3))
    w, h    = float(m.group(4)), float(m.group(5))
    obs.append((cx, cy, yaw_o, w, h))

# Start plotting
fig, ax = plt.subplots(figsize=(6,6))
ax.plot(x, y, '-o', label='RRT* Dubins path')

# Arrows for heading
dx, dy = np.cos(yaw), np.sin(yaw)
ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy',
          scale=5, width=0.005)

# Draw each obstacle as a rotated rectangle
for i, (cx, cy, yaw_o, w, h) in enumerate(obs):
    # Rectangle corners in local frame
    corners = np.array([
        [-w/2, -h/2],
        [ w/2, -h/2],
        [ w/2,  h/2],
        [-w/2,  h/2]
    ])
    # Rotation matrix
    R = np.array([
        [np.cos(yaw_o), -np.sin(yaw_o)],
        [np.sin(yaw_o),  np.cos(yaw_o)]
    ])
    # Rotate and translate
    corners_world = corners.dot(R.T) + np.array([cx, cy])
    poly = plt.Polygon(corners_world, closed=True,
                       edgecolor='r', fill=False,
                       linewidth=2,
                       label='Obstacle' if i==0 else None)
    ax.add_patch(poly)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Dubins RRT* Path with Obstacles')
ax.axis('equal')
ax.grid(True)
ax.legend()
plt.show()
