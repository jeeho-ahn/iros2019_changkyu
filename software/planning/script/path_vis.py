import numpy as np
import matplotlib.pyplot as plt

# ─── PASTE YOUR PATH HERE ────────────────────────────────────────────────────────
# Copy the printAsMatrix output (each line: x y yaw) into the raw string below.
raw = '''
1 1 0 
1.47943 1.12242 0.5 
1.8424 1.45906 0.952463 
2.10284 1.8858 1.02863 
2.45508 2.2333 0.528633 
2.92841 2.38539 0.20857 
3.3721 2.60443 0.70857 
3.76892 2.90857 0.650586 
4.10564 3.27633 0.883717 
4.42806 3.64256 0.826819 
4.84281 3.91241 0.326819 
5.25756 4.18226 0.826819 
5.50503 4.61316 1.16114 
5.79647 5.01601 0.844127 
6.12866 5.3897 0.844127 
6.46085 5.7634 0.844127 
6.79304 6.1371 0.844127 
7.12524 6.51079 0.844127 
7.45743 6.88449 0.844127 
7.78962 7.25818 0.844127 
8.15017 7.60446 0.757759 
8.47016 7.98822 0.894418 
'''
# ────────────────────────────────────────────────────────────────────────────────

# Parse it
lines = [l for l in raw.strip().splitlines() if l.strip()]
data = np.array([list(map(float, l.split())) for l in lines])
x, y, yaw = data[:,0], data[:,1], data[:,2]

# Plot
plt.figure(figsize=(6,6))
plt.plot(x, y, '-o', label='RRT* Dubins path')

# Arrows for heading
dx, dy = np.cos(yaw), np.sin(yaw)
plt.quiver(x, y, dx, dy,
           angles='xy', scale_units='xy', scale=5, width=0.005)

plt.xlabel('x')
plt.ylabel('y')
plt.title('Dubins RRT* Path')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()
