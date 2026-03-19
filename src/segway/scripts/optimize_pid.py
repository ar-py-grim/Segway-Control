#!/usr/bin/env python3

"""
Genetic Algorithm PID tuner for Segway
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import subprocess, time, math, random, datetime
from deap import base, creator, tools

# GA parameters
N_GEN = 8      # number of generations
POP_SIZE = 30  # individuals per generation
CX_PB = 0.5    # crossover probability
MUT_PB = 0.3   # mutation probability
TRIAL_DURATION = 5.0   # time (s) each individual is evaluated

# Gain search range [min, max]
KP_RANGE = (1.0, 150.0)
KI_RANGE = (0.0, 20.0)
KD_RANGE = (0.5, 50.0)

# Segway reset pose
RESET_CMD = [
    'gz', 'service', '-s', '/world/default/set_pose/blocking',
    '--reqtype', 'gz.msgs.Pose',
    '--reptype', 'gz.msgs.Boolean',
    '--timeout', '2000',
    '--req', "name: 'segway' position: {x: 9.0 y: 0.0 z: 0.05} "
             "orientation: {x: 0.0 y: 0.0 z: 0.707 w: 0.707}"
]

# shared state
current_pitch = 0.0
pitch_lock = __import__('threading').Lock()


class IMUListener(Node):
    def __init__(self):
        super().__init__('ga_imu_listener')
        self.create_subscription(Imu, '/imu', self._cb, 10)

    def _cb(self, msg):
        global current_pitch
        q0, q1, q2, q3 = msg.orientation.x, msg.orientation.y, \
                          msg.orientation.z, msg.orientation.w
        sinp = max(-1.0, min(1.0, 2.0*(q3*q1 - q2*q0)))
        with pitch_lock:
            current_pitch = math.asin(sinp)


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('ga_cmd_vel')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, linear_x: float):
        msg = Twist()
        msg.linear.x = max(-3.0, min(3.0, linear_x))
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())


def reset_robot():
    """Teleport segway back to upright start pose and wait until IMU confirms upright."""
    subprocess.run(RESET_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Wait until pitch is actually near zero, confirms reset took effect
    deadline = time.time()+5.0   # max 5s wait
    while time.time() < deadline:
        with pitch_lock:
            p = current_pitch
        if abs(p) < math.radians(5):   # within 5° of upright
            break
        time.sleep(0.05)
    else:
        print("robot did not reset to upright in 5s")


def evaluate(individual):
    """
    Run one trial, return fitness tuple:
      fitness[0] = total squared pitch error   (minimise → weight-1)
      fitness[1] = time survived upright       (maximise → weight+1)
    """
    Kp, Ki, Kd = individual
    time_interval = 0.005   # s

    reset_robot()

    integral       = 0.0
    last_err       = 0.0
    filtered_deriv = 0.0
    alpha          = 0.15
    pitch_error_sum = 0.0
    time_survived   = 0.0
    FALL_THRESHOLD  = math.radians(30)   # 30°

    steps = int(TRIAL_DURATION/time_interval)

    for _ in range(steps):
        t0 = time.time()

        with pitch_lock:
            pitch = current_pitch

        if abs(pitch) > FALL_THRESHOLD:
            break

        error = pitch

        integral+= (error + last_err)*time_interval/2
        integral  = max(-2.0, min(2.0, integral))

        raw_deriv = (error-last_err)/time_interval
        filtered_deriv = alpha*raw_deriv + (1.0-alpha)*filtered_deriv

        vel = Kp*error + Ki*integral + Kd*filtered_deriv
        cmd_pub.send(vel)

        last_err = error
        pitch_error_sum+= error ** 2
        time_survived+= time_interval

        elapsed = time.time()-t0
        if elapsed < time_interval:
            time.sleep(time_interval-elapsed)

    cmd_pub.stop()
    print(f"  Kp={Kp:.1f} Ki={Ki:.2f} Kd={Kd:.1f} | "
          f"survived={time_survived:.2f}s  pitch_err²={pitch_error_sum:.4f}")
    return pitch_error_sum, time_survived


# DEAP setup
creator.create("FitnessGA", base.Fitness, weights=(-1.0, 1.0))
creator.create("Individual", list, fitness=creator.FitnessGA)

toolbox = base.Toolbox()

def random_individual():
    return creator.Individual([
        random.uniform(*KP_RANGE),
        random.uniform(*KI_RANGE),
        random.uniform(*KD_RANGE)
])

toolbox.register("individual",  lambda: random_individual())
toolbox.register("population",  tools.initRepeat, list, toolbox.individual)
toolbox.register("evaluate",    evaluate)
toolbox.register("mate",        tools.cxBlend, alpha=0.2)
toolbox.register("mutate",      tools.mutGaussian, mu=0,
                                sigma=[10.0, 1.0, 3.0], indpb=0.3)
toolbox.register("select",      tools.selTournament, tournsize=3)


def clamp_individual(ind):
    ind[0] = max(KP_RANGE[0], min(KP_RANGE[1], ind[0]))
    ind[1] = max(KI_RANGE[0], min(KI_RANGE[1], ind[1]))
    ind[2] = max(KD_RANGE[0], min(KD_RANGE[1], ind[2]))


if __name__ == '__main__':
    rclpy.init()

    imu_node = IMUListener()
    cmd_pub  = CmdVelPublisher()

    import threading
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(imu_node)
    executor.add_node(cmd_pub)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0)
    random.seed(datetime.datetime.now().isoformat())

    print(f"Genetic PID Tuner — {POP_SIZE} individuals x {N_GEN} generations")
    print(f"Each trial: {TRIAL_DURATION}s  |  Total trials: ~{POP_SIZE*N_GEN}")
    print(f"\n")

    pop = toolbox.population(n=POP_SIZE)

    # Evaluate initial population
    print("Generation 0")
    for ind in pop:
        ind.fitness.values = toolbox.evaluate(ind)

    for gen in range(1, N_GEN+1):
        print(f"\n Generation {gen}")

        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        # Crossover
        for c1, c2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CX_PB:
                toolbox.mate(c1, c2)
                del c1.fitness.values
                del c2.fitness.values

        # Mutation
        for mutant in offspring:
            if random.random() < MUT_PB:
                toolbox.mutate(mutant)
                clamp_individual(mutant)
                del mutant.fitness.values

        # Evaluate only changed individuals
        invalid = [ind for ind in offspring if not ind.fitness.valid]
        for ind in invalid:
            ind.fitness.values = toolbox.evaluate(ind)

        pop[:] = offspring

        # Stats
        fits = [ind.fitness.values[1] for ind in pop]   # survival time
        print(f"  Survival time min:{min(fits):.2f}s  max:{max(fits):.2f}s  "
              f"avg:{sum(fits)/len(fits):.2f}s")

    # Filter out individuals that survived=0s, pitch_err=0
    valid = [ind for ind in pop if ind.fitness.values[1] > 0.0]
    if not valid:
        valid = pop
    best = tools.selBest(valid, 1)[0]
    print(f"\n")
    print(f"BEST INDIVIDUAL:")
    print(f"  Kp = {best[0]:.4f}")
    print(f"  Ki = {best[1]:.4f}")
    print(f"  Kd = {best[2]:.4f}")
    print(f"  Fitness: pitch_err\u00B2={best.fitness.values[0]:.4f}  "
          f"survived={best.fitness.values[1]:.2f}s")

    rclpy.shutdown()
    spin_thread.join()