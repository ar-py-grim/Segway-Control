#!/usr/bin/env python3

import rclpy, threading, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from pynput import keyboard

kp, ki, kd = 20.29, 14.67, 0.5
dt = 0.005
PITCH_STEP = 0.02
TURN_SPEED = 2.0

target_pitch = 0.0
target_w = 0.0
pitch = 0.0
held_keys = set()
keys_lock = threading.Lock()

def on_press(key):
    with keys_lock: held_keys.add(key)
    if key == keyboard.KeyCode.from_char('q'):
        rclpy.shutdown()

def on_release(key):
    with keys_lock: held_keys.discard(key)

class Segway(Node):
    def __init__(self):
        super().__init__('segway_controller')
        self.integral = 0.0
        self.last_err = 0.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(dt, self.update)

    def update(self):
        global target_pitch, target_w

        with keys_lock: held = frozenset(held_keys)
        target_pitch = PITCH_STEP  if keyboard.KeyCode.from_char('w') in held else \
                      -PITCH_STEP  if keyboard.KeyCode.from_char('s') in held else 0.0
        target_w     = TURN_SPEED  if keyboard.KeyCode.from_char('a') in held else \
                      -TURN_SPEED  if keyboard.KeyCode.from_char('d') in held else 0.0

        err = pitch - target_pitch
        self.integral+= (err+self.last_err)*dt/2
        vel = kp*err + ki*self.integral + kd*(err-self.last_err)/dt
        self.last_err = err

        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = target_w
        self.pub.publish(msg)

class IMU(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.create_subscription(Imu, '/imu', self.cb, 10)

    def cb(self, msg):
        global pitch
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        pitch = math.asin(max(-1.0, min(1.0, 2.0*(w*y - z*x))))


if __name__ == '__main__':

    rclpy.init()
    keyboard.Listener(on_press=on_press, on_release=on_release).start()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(Segway())
    executor.add_node(IMU())

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    print("W/S forward/back  A/D turn  Q quit")
    try:
        while rclpy.ok(): t.join(timeout=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): rclpy.shutdown()
        t.join()