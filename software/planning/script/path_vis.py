import json
import numpy as np
import matplotlib.pyplot as plt

# ─── PASTE YOUR PATH HERE ────────────────────────────────────────────────────────
raw = '''
{
  "robot": {"width":0.3, "forward":0.4, "backward":0.2},
  "obstacles": [
    {"cx":2,   "cy":2,   "yaw":0.785398, "width":0.3, "height":0.2},
    {"cx":3,   "cy":1,   "yaw":0,        "width":0.3, "height":0.3},
    {"cx":2.2, "cy":1.7, "yaw":0,        "width":0.3, "height":0.3}
  ],
  "solutionFound": true,
  "path": [
    [1,1,0],
    [1.49473,1.06642,0.178186],
    [1.99069,1.12956,0.122228],
    [2.46593,1.27633,0.476838],
    [2.90543,1.50808,0.620403],
    [2.90543,1.50808,0.620403],
    [3.25278,1.86408,0.975013],
    [3.4549,2.31855,1.32962],
    [3.48665,2.81492,1.68423],
    [3.35248,3.29491,1.92519],
    [3.35248,3.29491,1.92519],
    [3.17897,3.76384,1.92519],
    [3.02462,4.23766,1.75795],
    [3.02444,4.2386,1.75726],
    [3,4.5,1.5708]
  ]
}
'''

# Parse JSON
cfg       = json.loads(raw)
robot     = cfg["robot"]
obstacles = cfg["obstacles"]
path      = np.array(cfg["path"])

# Extract path coords & headings
x, y, yaw = path[:,0], path[:,1], path[:,2]

# Begin plotting
fig, ax = plt.subplots(figsize=(6,6))
ax.plot(x, y, '-o', color='k', label='RRT* Dubins path')

# Arrows for heading
dx, dy = np.cos(yaw), np.sin(yaw)
ax.quiver(x, y, dx, dy,
          angles='xy', scale_units='xy', scale=5,
          width=0.005, color='k')

# Draw obstacles
for i, o in enumerate(obstacles):
    cx, cy = o["cx"], o["cy"]
    yaw_o  = o["yaw"]
    w, h   = o["width"], o["height"]
    # rectangle corners in local frame
    corners = np.array([
        [-w/2, -h/2],
        [ w/2, -h/2],
        [ w/2,  h/2],
        [-w/2,  h/2]
    ])
    R = np.array([[np.cos(yaw_o), -np.sin(yaw_o)],
                  [np.sin(yaw_o),  np.cos(yaw_o)]])
    world_corners = corners.dot(R.T) + np.array([cx, cy])
    poly = plt.Polygon(world_corners, closed=True,
                       edgecolor='r', fill=False, linewidth=2,
                       label='Obstacle' if i==0 else None)
    ax.add_patch(poly)

# Draw robot footprint at every waypoint
w   = robot["width"]
fwd = robot["forward"]
bwd = robot["backward"]
# local corners of robot (forward/backward, left/right)
robot_corners = np.array([
    [  fwd,  w/2],
    [  fwd, -w/2],
    [ -bwd, -w/2],
    [ -bwd,  w/2]
])
for xi, yi, yawi in zip(x, y, yaw):
    Rr = np.array([[np.cos(yawi), -np.sin(yawi)],
                   [np.sin(yawi),  np.cos(yawi)]])
    rc_world = robot_corners.dot(Rr.T) + np.array([xi, yi])
    patch = plt.Polygon(rc_world, closed=True,
                        edgecolor='b', facecolor='b',
                        alpha=0.1, linewidth=1)
    ax.add_patch(patch)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_title('Dubins RRT* Path with Obstacles and Robot Trace')
ax.axis('equal')
ax.grid(True)
ax.legend()
plt.show()