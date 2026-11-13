"""Read-only geometric reachability diagnosis, isolated from navigation inputs."""
import json
from pathlib import Path
import sys
from collections import deque
import mujoco
import numpy as np

project = Path('F:/airace/lite2_sim')
sys.path.insert(0, str(project))
sys.path.insert(0, str(project / 'controllers/sensor_capture'))
from mujoco_navigation import MujocoVelocityExecutor, GROUPS

directory = Path(sys.argv[1])
config = json.loads((directory / 'config_snapshot.json').read_text())
model = mujoco.MjModel.from_xml_path(str(directory / 'scene.xml'))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)
executor = MujocoVelocityExecutor(model, data, config)
xs = np.arange(-5.55, 5.6, .1)
ys = np.arange(-4.55, 4.6, .1)
height = np.full((len(ys), len(xs)), np.nan)
for row, y in enumerate(ys):
    for col, x in enumerate(xs):
        normal = np.zeros(3)
        d = mujoco.mj_ray(model, data, np.array([x,y,.42]), np.array([0.,0.,-1.]), GROUPS, True, -1, None, normal)
        z = .42-d
        if d >= 0 and normal[2] > np.cos(np.radians(15)) and any(not executor.intersects([x,y,yaw], z, 0, 0) for yaw in np.arange(0, np.pi, np.pi/4)):
            height[row,col] = z
start = (int(np.argmin(abs(ys-config['robot_position_m'][1]))), int(np.argmin(abs(xs-config['robot_position_m'][0]))))
goal = (int(np.argmin(abs(ys-config['navigation']['goal_xy_m'][1]))), int(np.argmin(abs(xs-config['navigation']['goal_xy_m'][0]))))
queue = deque([start])
seen = {start}
while queue:
    row,col = queue.popleft()
    for dy,dx in ((1,0),(-1,0),(0,1),(0,-1)):
        r,c = row+dy,col+dx
        if 0 <= r < len(ys) and 0 <= c < len(xs) and (r,c) not in seen and np.isfinite(height[r,c]) and abs(height[r,c]-height[row,col]) <= .08:
            queue.append((r,c))
            seen.add((r,c))
print(json.dumps({'reachable':goal in seen,'seen_cells':len(seen),'free_cells':int(np.isfinite(height).sum()),'bounds':[[float(min(xs[c] for r,c in seen)),float(max(xs[c] for r,c in seen))],[float(min(ys[r] for r,c in seen)),float(max(ys[r] for r,c in seen))]]}))
