import math
import PyKDL as kdl
import abb_motion_program_exec as abb
import time
import numpy as np
from motion_3 import *
from abb_robot_client.egm import EGM
import matplotlib.pyplot as plt
from Jr3Manager import JR3Manager
import collections
import signal
import threading
from matplotlib.animation import FuncAnimation
import time
import matplotlib
matplotlib.use('QtAgg')

STEPS = 100
TIME_INTERVAL = 0.01  # [s]

# Configuración inicial de ABB y EGM
mm = abb.egm_minmax(-1e-3, 1e-3)
corr_frame = abb.pose([0, 0, 0], [1, 0, 0, 0])
corr_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
sense_frame = abb.pose([0, 0, 0], [1, 0, 0, 0])
sense_fr_type = abb.egmframetype.EGM_FRAME_WOBJ
egm_offset = abb.pose([0, 0, 0], [1, 0, 0, 0])
egm_config = abb.EGMPoseTargetConfig(corr_frame, corr_fr_type, sense_frame, sense_fr_type, mm, mm, mm, mm, mm, mm, 1000, 1000)

r1 = abb.robtarget([500, 200, 0], [0, 0, 1, 0], abb.confdata(0, 0, -1, 1), [0]*6)
tool = abb.tooldata(True, abb.pose([125.800591275, 0, 391.268161315], [0.898794046, 0, 0.438371147, 0]), abb.loaddata(3, [0, 0, 100], [0, 1, 0, 0], 0, 0, 0))
mp = abb.MotionProgram(egm_config=egm_config, tool=tool)
mp.EGMRunPose(10, 0.05, 0.05, egm_offset)

client = abb.MotionProgramExecClient(base_url="http://127.0.0.1:80")
lognum = client.execute_motion_program(mp, wait=False)

# Inicialización del sensor JR3
jr3 = JR3Manager('COM5', 115200)
jr3.start(200, 10000)
time.sleep(1)
jr3.zero_offs()

# Parámetros de la trayectoria
alpha = -90 * (np.pi / 180)
p1r = kdl.Frame(kdl.Vector(550, 200, 0))
p2r = kdl.Frame(kdl.Vector(650, 200, 0))
p1c = kdl.Vector(650, 150, 0)
p3r = kdl.Frame(kdl.Vector(700, 150, 0))
p4r = kdl.Frame(kdl.Vector(700, -150, 0))
p2c = kdl.Vector(650, -150, 0)
p5r = kdl.Frame(kdl.Vector(650, -200, 0))
p6r = kdl.Frame(kdl.Vector(550, -200, 0))
p3c = kdl.Vector(550, -150, 0)
p7r = kdl.Frame(kdl.Vector(500, -150, 0))
p8r = kdl.Frame(kdl.Vector(500, 150, 0))
p4c = kdl.Vector(550, 150, 0)

path1 = PathLine(p1r, p2r)
path2 = PathCircle(p2r, p1c, alpha)
path3 = PathLine(p3r, p4r)
path4 = PathCircle(p4r, p2c, alpha)
path5 = PathLine(p5r, p6r)
path6 = PathCircle(p6r, p3c, alpha)
path7 = PathLine(p7r, p8r)
path8 = PathCircle(p8r, p4c, alpha)

profile1 = VelocityProfileRectangular(100.0)
profile2 = VelocityProfileRectangular(100.0)
profile3 = VelocityProfileRectangular(100.0)
profile4 = VelocityProfileRectangular(100.0)
profile5 = VelocityProfileRectangular(100.0)
profile6 = VelocityProfileRectangular(100.0)
profile7 = VelocityProfileRectangular(100.0)
profile8 = VelocityProfileRectangular(100.0)

traj1 = TrajectorySegment(path1, profile1, 2.0)
traj2 = TrajectorySegment(path2, profile2, 1.0)
traj3 = TrajectorySegment(path3, profile3, 2.0)
traj4 = TrajectorySegment(path4, profile4, 1.0)
traj5 = TrajectorySegment(path5, profile5, 2.0)
traj6 = TrajectorySegment(path6, profile6, 1.0)
traj7 = TrajectorySegment(path7, profile7, 2.0)
traj8 = TrajectorySegment(path8, profile8, 1.0)


def is_within_workspace(p):
    x_min, x_max = 400, 700
    y_min, y_max = -200, 200
    z_min, z_max = -50, 50
    return x_min <= p[0] <= x_max and y_min <= p[1] <= y_max and z_min <= p[2] <= z_max

# Control del EGM y envío de posiciones al robot
egm = EGM()

t = np.arange(0, STEPS)
values = [collections.deque(np.zeros(t.shape), maxlen=STEPS) for _ in range(6)]
last_value = [0 for _ in range(6)]
limits = [(0, 0) for _ in range(2)]

should_stop = False

def handler(signum, frame):
    global should_stop
    should_stop = True

signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)

fig, axes = plt.subplots()
mngr = plt.get_current_fig_manager()
# to put it into the upper left corner for example:
mngr.window.setGeometry(850,10,640, 545)

axes.set_title('FUERZAS')
axes.set_animated(True)
# Añadir grid
axes.grid(True)

(ln_fz,) = axes.plot(values[2], label='z', color='blue')


plt.show(block=False)
plt.pause(0.1)

bg = fig.canvas.copy_from_bbox(fig.bbox)

fig.draw_artist(axes)

# https://matplotlib.org/stable/users/explain/animations/blitting.html + https://stackoverflow.com/a/15724978
fig.canvas.blit(fig.bbox)
def do_draw():
    global should_stop

    while not should_stop:
        for i in range(len(values)):
            values[i].popleft()
            values[i].append(last_value[i])

        fig.canvas.restore_region(bg)
        
        ln_fz.set_ydata(values[2])

        axes.set_ylim(limits[0][0], limits[0][1])

        fig.draw_artist(axes)

        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()

        time.sleep(TIME_INTERVAL)

thread = threading.Thread(target=do_draw)
thread.start()

start_time = time.time()

while True:
    traj = TrajectoryComposite()
    traj.add_segment(traj1)
    traj.add_segment(traj2)
    traj.add_segment(traj3)
    traj.add_segment(traj4)
    traj.add_segment(traj5)
    traj.add_segment(traj6)
    traj.add_segment(traj7)
    traj.add_segment(traj8)

    start = time.perf_counter()

    while (time.perf_counter() - start) < traj.duration():
        now = time.perf_counter() - start
        res, feedback = egm.receive_from_robot(timeout=0.05)
        if res:
            H = traj.position(now)
            p = np.array([H.p.x(), H.p.y(), H.p.z()])
            if is_within_workspace(p):
                success, forces, torques, _ = jr3.read()
                if success:
                    p[2] += forces[2] * 10
                    last_value[0] = forces[0]
                    last_value[1] = forces[1]
                    last_value[2] = forces[2]
                    limits[0] = (min([limits[0][0]] + last_value[0:3]), max([limits[0][1]] + last_value[0:3]))
                    last_value[3] = torques[0]
                    last_value[4] = torques[1]
                    last_value[5] = torques[2]
                    limits[1] = (min([limits[1][0]] + last_value[3:6]), max([limits[1][1]] + last_value[3:6]))
                egm.send_to_robot_cart(p, [0, 0, 1, 0])
            else:
                print(f"Posición fuera del área de trabajo: {p}")
    
    del traj

