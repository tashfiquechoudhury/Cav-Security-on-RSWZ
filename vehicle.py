import numpy as np
import pandas as pd
import random


class Vehicle:

    def __init__(self, max_velocity, max_acceleration):
        self.max_v_x = max_velocity
        self.max_a_x = max_acceleration
        random.seed(max_velocity)

    def generate_trajectory_1D(self, total_duration, timestep, init_velocity):
        """
        Returns the velocity, position, and acceleration of the CAV at each timestep up until total_duration as a Pandas
        Dataframe in 1D.

        Format:
                time    position    velocity    acceleration
                 0
                 tau
                 2tau
                 3tau
                 ...
                 N

        where N = total_duration

        :param total_duration:
        :param timestep:
        :param init_velocity:
        """
        # Initialize velocity, acceleration, position, and time arrays.
        tau = timestep
        N = int(total_duration / tau) + 1
        t = np.zeros(N, dtype=np.float32)
        pos_x = np.zeros(N, dtype=np.float32)
        v_x = np.zeros(N, dtype=np.float32)
        a_x = np.zeros(N, dtype=np.float32)

        # Initialize initial velocity
        v_x[0] = init_velocity

        # Simulate trajectory of CAV
        for i in range(1, N):
            pos_x[i] = pos_x[i - 1] + t[i - 1] * v_x[i - 1] + (0.5 * a_x[i - 1] * (t[i - 1] ** 2))
            v_x[i] = v_x[i - 1] + t[i - 1] * a_x[i - 1]
            if v_x[i] > self.max_v_x:  # Make sure our velocity doesn't explode...
                v_x[i] = self.max_v_x
            a_x[i] = self.change_acceleration(a_x[i - 1])
            t[i] = t[i - 1] + tau

        # Aggregate and combine data into one dataframe.
        data = np.column_stack((t, pos_x, v_x, a_x))
        dataframe = pd.DataFrame(data, columns=['time', 'position', 'velocity', 'acceleration'])
        return dataframe

    def change_acceleration(self, prev_a_x):
        """
        Returns an updated acceleration based on the previous acceleration, current time, and the total duration
        of the trip. Uses a pseudo-random number generator to provide random scenarios of what acceleration looks like.
        (E.g., speeding up for some time, then slowing down)

        :param prev_a_x:
        """
        # We will have 5 scenarios (accelerate, decelerate, accelerate quickly then decelerate slowly, accelerate
        # slowly then decelerate quickly, accelerate moderately then decelerate moderately)

        scenario = int(random.random() * 4) + 1  # scenarios 1-5

        if scenario == 1:
            a_x = prev_a_x + 1
            if a_x > self.max_a_x:
                return self.max_a_x
            return a_x
        elif scenario == 2:
            return prev_a_x - 1
        elif scenario == 3:
            a_x = prev_a_x + 5
            if a_x > self.max_a_x:
                a_x = self.max_a_x
            a_x -= 0.5
            return a_x
        elif scenario == 4:
            a_x = prev_a_x + 0.5
            if a_x > self.max_a_x:
                a_x = self.max_a_x
            a_x -= 5
            return a_x
        else:
            return prev_a_x

