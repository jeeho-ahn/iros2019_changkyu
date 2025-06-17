#!/usr/bin/env python3
import re
import sys
import warnings
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib.widgets import Slider

# Suppress the requests dependency warning
warnings.filterwarnings("ignore", message=".*RequestsDependencyWarning.*")

def parse_result(fp):
    """Parse actions and path blocks from the planner result file."""
    lines = open(fp).read().splitlines()
    actions, path = [], []
    i, n = 0, len(lines)

    while i < n:
        line = lines[i].strip()
        # parse actions block
        if line.startswith('actions:'):
            while i < n and '[' not in lines[i]:
                i += 1
            i += 1
            while i < n and not lines[i].strip().startswith(']'):
                row = lines[i].strip()
                m = re.match(
                    r'\[\s*(\d+)\s*,\s*(\d+)\s*,\s*(-?\d+)\s*,\s*'
                    r'([-\d\.eE]+)\s*,\s*([-\d\.eE]+)\s*,\s*([-\d\.eE]+)\s*\]',
                    row
                )
                if m:
                    t, idx, idx2, x, y, yaw = m.groups()
                    actions.append({
                        'type': int(t),
                        'obj':  int(idx),
                        'x':   float(x),
                        'y':   float(y),
                        'yaw': float(yaw)
                    })
                i += 1
        # parse path block (unused for slider but needed for overall)
        elif line.startswith('path:'):
            while i < n and '[' not in lines[i]:
                i += 1
            i += 1
            while i < n and not lines[i].strip().startswith(']'):
                row = lines[i].strip()
                if row.startswith('['):
                    nums = re.findall(r'-?[\d\.eE]+', row)
                    path.append([float(val) for val in nums])
                i += 1
        else:
            i += 1

    return actions, path

def draw_overall(ax, path):
    """Draw static overall trajectories on ax."""
    if not path:
        return
    n_objs = (len(path[0]) - 1) // 3
    init = path[0][1:1 + 3*n_objs]
    goal = path[-1][1:1 + 3*n_objs]
    traj = {o:([],[]) for o in range(1, n_objs+1)}
    for state in path:
        for o in range(1, n_objs+1):
            traj[o][0].append(state[1+3*(o-1)])
            traj[o][1].append(state[2+3*(o-1)])
    colors = plt.cm.tab10.colors
    ax.set_aspect('equal','box')
    ax.set_title("Overall Object Paths")
    for o,(xs,ys) in traj.items():
        c = colors[(o-1)%len(colors)]
        ax.plot(xs, ys, '-', color=c)
        ix, iy = init[3*(o-1):3*(o-1)+2]
        gx, gy = goal[3*(o-1):3*(o-1)+2]
        ax.scatter([ix],[iy],marker='o',color=c)
        ax.text(ix, iy, f"S{o}", color=c)
        ax.scatter([gx],[gy],marker='X',color=c)
        ax.text(gx, gy, f"G{o}", color=c)

def draw_frame(ax, actions, frame, xlim, ylim):
    """Redraw ax with arrows and oriented rectangles up to a given frame index."""
    ax.clear()
    ax.set_aspect('equal','box')
    ax.set_title(f"Action Sequence (frame {frame}/{len(actions)-1})")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    colors = plt.cm.tab10.colors
    prev = actions[0]
    ax.scatter(prev['x'], prev['y'], color=colors[(prev['obj']-1)%len(colors)])
    for i in range(1, frame+1):
        a = actions[i]
        c = colors[(a['obj']-1)%len(colors)]
        ax.annotate(
            '',
            xy=(a['x'], a['y']),
            xytext=(prev['x'], prev['y']),
            arrowprops=dict(arrowstyle='->', color=c, lw=2)
        )
        if a['type'] == 1:
            w,h = 0.3, 0.3
            rect = patches.Rectangle(
                (a['x']-w/2, a['y']-h/2),
                w, h, fill=False, lw=2, color=c
            )
            angle = math.degrees(a['yaw'])
            trans = transforms.Affine2D().rotate_deg_around(a['x'], a['y'], angle) + ax.transData
            rect.set_transform(trans)
            ax.add_patch(rect)
        prev = a

def main(result_file):
    actions, path = parse_result(result_file)
    if not actions or not path:
        print("No actions or path data. Check your result file.")
        sys.exit(1)

    xs = [a['x'] for a in actions]
    ys = [a['y'] for a in actions]
    margin = 0.5
    xlim = (min(xs)-margin, max(xs)+margin)
    ylim = (min(ys)-margin, max(ys)+margin)

    fig = plt.figure(figsize=(12,6))
    ax1 = fig.add_subplot(1,2,1)
    ax2 = fig.add_subplot(1,2,2)
    draw_overall(ax1, path)

    # Slider below axes:
    ax_slider = fig.add_axes([0.25, 0.02, 0.5, 0.03])
    slider = Slider(ax_slider, 'Frame', 0, len(actions)-1, valinit=0, valstep=1)

    # Initial draw
    draw_frame(ax2, actions, 0, xlim, ylim)

    # Update on slider change
    slider.on_changed(lambda val: draw_frame(ax2, actions, int(val), xlim, ylim))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python visualize_plan_with_slider.py path/to/result.res")
        sys.exit(1)
    main(sys.argv[1])