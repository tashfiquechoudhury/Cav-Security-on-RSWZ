import numpy as np
import pandas as pd
import random as r


class Vehicle:
    """
    An object that represents a Connected and Autonomous Vehicle (CAV). We can generate trajectories of the vehicle
    based on various parameters.
    """

    def __init__(self, v_max, a_max, model, make):
        self.max_v = v_max
        self.max_a = a_max
        self.model = model
        self.make = make

    def trajectory(self, v_init, timestep, duration, seed):
        """
        Returns the velocity, position, and acceleration of the CAV at each timestep up until duration as three numpy
        arrays.

        :param v_init:
        :param timestep:
        :param duration:
        :param seed:
        """
        # Initialize velocity (v), acceleration (a), position (x), and time (t) arrays,
        # and pseudorandom number generator.
        tau = timestep
        N = int(duration / tau) + 1
        t = np.zeros(N, dtype=np.float32)
        x = np.zeros(N, dtype=np.float32)
        v = np.zeros(N, dtype=np.float32)
        a = np.zeros(N, dtype=np.float32)
        r.seed(seed)

        # Initialize entry 0 of velocity array with init_v
        v[0] = v_init

        # Break down our trajectory into three phases:
        # acceleration phase, random trajectory phase, and deceleration phase.

        # Our acceleration phase should be last from t = 1 up until t = [N/5, N/4]
        # At most, 25% of the trip is the vehicle accelerating
        # ACCELERATION PHASE:
        acc_t_start = 1
        acc_t_end = (int(r.uniform((len(t) - 1) / 5, (len(t) - 1) / 4))) + 1
        acc_duration = duration / acc_t_end
        acc = self.acc(int(r.uniform(1, 4)), v_init)
        for i in range(acc_t_start, acc_t_end):
            a[i] = acc(curr_t=i, duration=acc_duration)
            x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i - 1] * (tau ** 2))
            v[i] = v[i - 1] + tau * a[i]
            t[i] = t[i - 1] + tau

        # Moving in constant velocity now.
        # RANDOM TRAJECTORY PHASE:
        ran_t_start = acc_t_end
        ran_t_end = 1

        # Given the current velocity we're traveling and the time remaining from the trip, we need to calculate
        # the value of acceleration to decelerate our current velocity such that it reaches 0 at the end.
        # We will be decelerating the last 10th of the trip
        # DECELERATION PHASE:
        return a

    def acc(self, choice, v_init):
        """
        A HoF that will return the acceleration scenario for the CAV during the acceleration phase. With the given
        acceleration function, the CAV will accelerate up to the desired velocity (a value between init_v + 5 and
        the max velocity)

        :param choice:
        :param v_init:
        """

        v_des = int(r.uniform(v_init + 5, self.max_v))  # v_des = [init_v + 5, max_v]
        v_fast = int(r.uniform((3 * v_des) / 4, (7 * v_des) / 8))  # v_fast = [(3/4) * v_des, (7/8) * v_des]
        v_slow = int(r.uniform(v_des / 4, v_des / 2))  # v_slow = [(1/4) * v_des, (1/2) * v_des]

        def acc_quickly_then_slowly(**params):
            """
            A function that will simulate the acceleration of a car speeding up quickly then speeding up slowly. First,
            the car will speed up quickly for half the duration then speed up slowly for the rest of the duration.
            """
            # We want to a_fast to reach some v s.t. v <= max_v
            a_fast = (v_fast - v_init) / (params['duration'] / 2)
            a_slow = (v_des - v_fast) / (params['duration'] / 2)
            if params['curr_t'] <= params['duration'] / 2:
                return a_fast
            else:
                return a_slow

        def acc_slowly_then_quickly(**params):
            """
            A function that will simulate the acceleration of a car speeding up slowly then speeding up quickly. First,
            the car will speed up slowly for half of the duration then speed up quickly for the rest of the duration.
            """
            a_slow = (v_slow - v_init) / (params['duration'] / 2)
            a_fast = (v_des - v_slow) / (params['duration'] / 2)
            if params['curr_t'] <= params['duration'] / 2:
                return a_slow
            else:
                return a_fast

        def acc_quickly(**params):
            """
            A function that will simulate the acceleration of a car speeding up quickly. The car will be speeding up
            quickly for the entire duration of the acceleration phase.
            """
            return (v_fast - v_init) / params['duration']

        def acc_slowly(**params):
            """
            A function that will simulate the acceleration of a car speeding up slowly. The car will be speeding up
            slowly for the entire duration of the acceleration phase.
            """
            return (v_slow - v_init) / params['duration']

        if choice == 1:
            return acc_quickly_then_slowly
        elif choice == 2:
            return acc_slowly_then_quickly
        elif choice == 3:
            return acc_quickly
        elif choice == 4:
            return acc_slowly
