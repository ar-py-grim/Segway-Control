#!/usr/bin/env python3

import rclpy, threading, math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAX_POINTS = 2000

log_time   = []
log_pitch  = []
log_cmdvel = []
t_start    = None
data_lock  = threading.Lock()


class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_results')
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmdvel_cb, 10)
        self._last_cmdvel = 0.0

    def imu_cb(self, msg):
        global t_start
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        pitch_rad = math.asin(max(-1.0, min(1.0, 2.0*(w*y - z*x))))

        now = self.get_clock().now().nanoseconds * 1e-9
        with data_lock:
            if t_start is None:
                t_start = now
            log_time.append(now - t_start)
            log_pitch.append(math.degrees(pitch_rad))
            log_cmdvel.append(self._last_cmdvel)
            if len(log_time) > MAX_POINTS:
                del log_time[0]
                del log_pitch[0]
                del log_cmdvel[0]

    def cmdvel_cb(self, msg):
        self._last_cmdvel = msg.linear.x


def main():
    rclpy.init()
    node = PlotNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(f"Segway PID Response  (Kp={__import__('segway_control').kp}  "
                 f"Ki={__import__('segway_control').ki}  Kd={__import__('segway_control').kd})"
                 if False else "Segway Live PID Response")

    line_pitch,  = ax1.plot([], [], color='steelblue')
    line_cmdvel, = ax2.plot([], [], color='tomato')

    ax1.axhline(0, color='black', linewidth=0.5)
    ax1.set_ylabel("Pitch (deg)")
    ax1.set_ylim(-35, 35)
    ax1.grid(True, alpha=0.3)

    ax2.axhline(0, color='black', linewidth=0.5)
    ax2.set_ylabel("cmd_vel (m/s)")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True, alpha=0.3)

    def update(_):
        with data_lock:
            t = list(log_time)
            p = list(log_pitch)
            v = list(log_cmdvel)

        if not t:
            return line_pitch, line_cmdvel

        line_pitch.set_data(t, p)
        line_cmdvel.set_data(t, v)

        ax1.set_xlim(max(0, t[-1]-30), t[-1]+1)
        ax2.set_xlim(max(0, t[-1]-30), t[-1]+1)
        ax2.set_ylim(min(v)-0.5, max(v)+0.5)

        return line_pitch, line_cmdvel

    ani = animation.FuncAnimation(fig, update, interval=100, blit=True)

    plt.tight_layout()
    plt.rcParams['keymap.save'].remove('s')
    plt.show()

    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()