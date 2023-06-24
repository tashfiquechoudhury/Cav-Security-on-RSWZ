import numpy as np
import pandas as pd

from acceleration import Acceleration


class Trajectory:  # Trajectory HAS A acceleration object

    def __init__(self, max_v, max_a, seed):
        self.max_v = max_v
        self.max_a = max_a
        self.a_x = Acceleration(seed, max_a, max_v)

    def trajectory(self, total_duration, timestep, init_v):
        """
        Returns the velocity, position, and acceleration of the CAV at each timestep up until total_duration as a Pandas
        Dataframe.

        Format:
            time    position    velocity    acceleration\n
            0\n
            tau\n
            2tau\n
            3tau\n
            ...\n
            N\n
        where N = total_duration

        :param total_duration:
        :param timestep:
        :param init_v:
        """
        # Initialize velocity (v), acceleration (a), position (x), and time (t) arrays.
        tau = timestep
        N = int(total_duration / tau) + 1
        t = np.zeros(N, dtype=np.float32)
        x = np.zeros(N, dtype=np.float32)
        v = np.zeros(N, dtype=np.float32)
        a = np.zeros(N, dtype=np.float32)

        # Initialize initial velocity
        v[0] = init_v

        # Simulate trajectory of CAV
        for i in range(1, N):
            x[i] = x[i - 1] + t[i - 1] * v[i - 1] + (0.5 * a[i - 1] * (t[i - 1] ** 2))
            v[i] = v[i - 1] + t[i - 1] * a[i - 1]
            if v[i] > self.max_v:  # Make sure our velocity doesn't explode...
                v[i] = self.max_v
            a[i] = self.a_x.set_acceleration(total_duration, i, a[i - 1], v[i], x[i])
            t[i] = t[i - 1] + tau

        # Aggregate and combine data into one dataframe.
        data = np.column_stack((t, x, v, a))
        dataframe = pd.DataFrame(data, columns=['time', 'position', 'velocity', 'acceleration'])
        return dataframe
