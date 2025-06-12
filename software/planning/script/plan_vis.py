import re
import sys
import warnings
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import matplotlib.transforms as transforms

# Suppress the requests dependency warning
warnings.filterwarnings("ignore", message=".*RequestsDependencyWarning.*")

def parse_result(fp):
    """Parse actions and path blocks from the planner result file."""
    lines = open(fp).read().splitlines()
    actions, path = [], []
    i, n = 0, len(lines)

    while i < n:
        line = lines[i].strip()
        # ----- parse actions block -----
        if line.startswith('actions:'):
            # skip until the first '[' on this block
            while i < n and '[' not in lines[i]:
                i += 1
            i += 1
            # read each action line until closing ']'
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

        # ----- parse path block -----
        elif line.startswith('path:'):
            # skip until first '[' of the path list
            while i < n and '[' not in lines[i]:
                i += 1
            i += 1
            # read each path-state line until closing ']'
            while i < n and not lines[i].strip().startswith(']'):
                row = lines[i].strip()
                if row.startswith('['):
                    nums = re.findall(r'-?[\d\.eE]+', row)
                    path.append([float(val) for val in nums])
                i += 1

        else:
            i += 1

    return actions, path

def visualize(actions, path):
    """Draw static overall paths and animate the action sequence with oriented rectangles."""
    if not path:
        raise ValueError("No path data found in result file")

    # determine number of objects from path-state length
    n_objs = (len(path[0]) - 1) // 3

    # derive start & goal poses from the first and last path-state
    init = path[0][1:1 + 3*n_objs]
    goal = path[-1][1:1 + 3*n_objs]

    # build per-object trajectories
    traj = {o: ([], []) for o in range(1, n_objs+1)}
    for state in path:
        for o in range(1, n_objs+1):
            x = state[1 + 3*(o-1)]
            y = state[2 + 3*(o-1)]
            traj[o][0].append(x)
            traj[o][1].append(y)

    colors = plt.cm.tab10.colors
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # -- Static overall paths --
    ax1.set_aspect('equal', 'box')
    ax1.set_title("Overall Object Paths")
    for o, (xs, ys) in traj.items():
        c = colors[(o-1) % len(colors)]
        ax1.plot(xs, ys, '-', color=c, label=f"obj{o}")
        ix, iy = init[3*(o-1):3*(o-1)+2]
        gx, gy = goal[3*(o-1):3*(o-1)+2]
        ax1.scatter([ix], [iy], marker='o', color=c)
        ax1.text(ix, iy, f"S{o}", color=c)
        ax1.scatter([gx], [gy], marker='X', color=c)
        ax1.text(gx, gy, f"G{o}", color=c)
    ax1.legend()

    # -- Action sequence animation --
    ax2.set_aspect('equal', 'box')
    ax2.set_title("Action Sequence")
    xs = [a['x'] for a in actions]
    ys = [a['y'] for a in actions]
    margin = 0.5
    ax2.set_xlim(min(xs) - margin, max(xs) + margin)
    ax2.set_ylim(min(ys) - margin, max(ys) + margin)

    artists = []
    prev = actions[0]
    artists.append(ax2.scatter(prev['x'], prev['y'], color=colors[(prev['obj']-1) % len(colors)]))

    # intervals: transit slower, transfer faster
    transit_ms = 800
    transfer_ms = 100  # faster transfer

    def update(i):
        nonlocal prev, anim
        a = actions[i]
        c = colors[(a['obj']-1) % len(colors)]
        # draw arrow
        arr = ax2.annotate(
            '',
            xy=(a['x'], a['y']),
            xytext=(prev['x'], prev['y']),
            arrowprops=dict(arrowstyle='->', color=c, lw=2)
        )
        artists.append(arr)

        if a['type'] == 1:
            # oriented rectangle centered at (x, y)
            w, h = 0.3, 0.3
            rect = patches.Rectangle(
                (a['x'] - w/2, a['y'] - h/2),
                w, h,
                fill=False, lw=2, color=c
            )
            # rotate around center
            angle = math.degrees(a['yaw'])
            trans = transforms.Affine2D().rotate_deg_around(a['x'], a['y'], angle) + ax2.transData
            rect.set_transform(trans)
            ax2.add_patch(rect)
            artists.append(rect)
            anim.event_source.interval = transfer_ms
        else:
            anim.event_source.interval = transit_ms

        prev = a
        return artists

    anim = FuncAnimation(
        fig, update,
        frames=range(1, len(actions)),
        interval=transit_ms,
        blit=True,
        repeat=False
    )

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python visualize_plan_oriented.py path/to/result.res")
        sys.exit(1)
    actions, path = parse_result(sys.argv[1])
    visualize(actions, path)


# Save the updated script
with open('/mnt/data/visualize_plan_oriented.py', 'w') as f:
    f.write(script)

print("Saved the oriented visualization script to /mnt/data/visualize_plan_oriented.py")