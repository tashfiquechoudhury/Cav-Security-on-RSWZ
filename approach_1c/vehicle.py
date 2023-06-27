import numpy as np
import pandas as pd
import random as r


class Vehicle:
    """
    An object that represents a Connected and Autonomous Vehicle (CAV). We can generate trajectories of the vehicle
    based on various parameters.
    """

    def __init__(self, max_v, max_a, model, make):
        self.max_v = max_v
        self.max_a = max_a
        self.model = model
        self.make = make

    def trajectory(self, init_v, timestep, duration, seed):
        """
        Returns the velocity, position, and acceleration of the CAV at each timestep up until duration as three numpy
        arrays.

        Format:
            time    position    velocity    acceleration\n
            0\n
            tau\n
            2tau\n
            3tau\n
            ...\n
            N\n
        where N = total_duration

        :param init_v:
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

        # Initialize initial velocity
        v[0] = init_v

        # Break down our trajectory into three phases:
        # acceleration phase, random trajectory phase, and deceleration phase.

        # Our acceleration phase should be last from t = 1 up until t = [N/5, N/4]
        # At most, 25% of the trip is the vehicle accelerating
        # ACCELERATION PHASE:
        t_1 = int(r.uniform((1 / 5) * len(t), (1 / 4) * len(t)))
        choice = int(r.uniform(1, 4))
        accelerator = self.accelerator(choice, init_v)
        for i in range(1, t_1 + 1):
            x[i] = x[i - 1] + t[i - 1] * v[i - 1] + (0.5 * a[i - 1] * (t[i - 1] ** 2))
            v[i] = v[i - 1] + t[i - 1] * a[i - 1]
            a[i] = accelerator(i, t_1)

        a[t_1 + 1] = 0  # Moving in constant velocity now.

    def accelerator(self, choice, init_v):
        """
        A HoF that will return the acceleration scenario for the CAV during the acceleration phase.

        :param choice:
        :param init_v:
        """

        desired_v = int(r.uniform(init_v + 5, self.max_v))
        v_fast = int(r.uniform((3 / 4) * desired_v, (7 / 8) * desired_v))
        v_slow = int(r.uniform((1 / 4) * desired_v, (1 / 2) * desired_v))

        def accelerate_quickly_then_slowly(curr_t, duration):
            """
            A function that will simulate the acceleration of a car speeding up quickly then speeding up slowly. First,
            the car will speed up quickly for half the duration then speed up slowly for the rest of the duration.

            :param curr_t:
            :param duration:
            """
            # We want to a_fast to reach some v s.t. v <= max_v
            a_fast = v_fast - init_v / (duration / 2)
            a_slow = desired_v - v_fast / (duration / 2)
            if curr_t <= duration / 2:
                return a_fast
            else:
                return a_slow

        def accelerate_slowly_then_quickly(curr_t, duration):
            """
            A function that will simulate the acceleration of a car speeding up slowly then speeding up quickly. First,
            the car will speed up slowly for half of the duration then speed up quickly for the rest of the duration.
            """
            a_slow = v_slow - init_v / (duration / 2)
            a_fast = desired_v - v_slow / (duration / 2)
            if curr_t <= duration / 2:
                return a_slow
            else:
                return a_fast

        def accelerate_quickly(curr_t, duration):
            """
            A function that will simulate the acceleration of a car speeding up quickly. The car will be speeding up
            quickly for the entire duration of the acceleration phase.
            """
            return v_fast - init_v / duration

        def accelerate_slowly(curr_t, duration):
            """
            A function that will simulate the acceleration of a car speeding up slowly. The car will be speeding up
            slowly for the entire duration of the acceleration phase.
            """
            return v_slow - init_v / duration

        if choice == 1:
            return accelerate_quickly_then_slowly
        elif choice == 2:
            return accelerate_slowly_then_quickly
        elif choice == 3:
            return accelerate_quickly
        elif choice == 4:
            return accelerate_slowly
