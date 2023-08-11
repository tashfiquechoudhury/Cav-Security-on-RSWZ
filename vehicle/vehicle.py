import numpy as np
import pandas as pd
import random as r


class Vehicle:
    """
    An object that represents a Connected and Autonomous Vehicle (CAV). We can generate trajectories of the vehicle
    based on various parameters.
    """

    def __init__(self, v_max, a_max):
        """
        The constructor for the CAV.

        :param v_max: The maximum velocity of the vehicle, in m/s (int).
        :param a_max: The maximum acceleration of the vehicle, in m/s^2 (int).
        """
        self.v_max = v_max
        self.a_max = a_max
        self.cache = None
        self.prev_action = None
        self.comms = []

    def trajectory(self, v_init, timestep, duration, v2i_comms, seed=0):
        """
        Reports the velocity, position, and acceleration of the CAV at each timestep up until duration as a dictionary.

        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param timestep: The timestep of the trip (int).
        :param duration: The duration of the trip, in seconds (int).
        :param seed: The seed of the trip (int).
        :param v2i_comms: V2I communications (list).
        """
        # Clear cache from previous trajectory
        self.prev_action = None
        self.cache = None
        self.comms = []  # self.comms = [( (v2i[i], [start, end, i where v = 0 or v = rs]))]

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
        acc_duration = (duration / ((len(t) - 1) / (acc_t_end - 1)))
        acc = self.acc_acc(int(r.uniform(0, 4)), v[0], acc_duration, acc_t_end - 1)
        for i in range(acc_t_start, acc_t_end):
            # Update trajectory information
            a[i] = acc(i)
            x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i] * (tau ** 2))
            v[i] = v[i - 1] + tau * a[i]
            t[i] = t[i - 1] + tau

        # Moving in constant velocity now.
        # RANDOM TRAJECTORY PHASE:
        ran_t_start = acc_t_end
        ran_t_end = int((9 / 10) * len(t)) + 1
        counter = 0
        i = ran_t_start

        # Lines 76-84 just represent the V2I comms that will be passed in the CAV during its trajectory.
        increments = [r.randint(50, 150) for _ in range(len(v2i_comms))]
        if v2i_comms:
            v2i = \
                {
                    ran_t_start + increments[i]: v2i_comms[i] for i in range(len(v2i_comms))
                }
        else:
            v2i = {}

        while i < ran_t_end:
            if i in v2i:
                # Read in the V2I communication
                # comm = ['RS', 'dist_to_WZ', 'speed_limit', 'len_of_WZ']
                # OR
                # comm = ['S', 'dist_to_WZ', 'duration']
                comm = v2i[i].split(",")
                start = i

                if comm[0] == 'RS':
                    curr_v = v[i - 1]
                    dist_to_WZ, des_v, dist_of_WZ = int(comm[1]), int(comm[2]), int(comm[3])

                    # To calculate the appropriate deceleration, we use the following kinematic equation
                    # a = (reduced_speed ** 2) - (speed ** 2) / ((2 * distance_to_WZ))
                    if curr_v > des_v:
                        dec = ((des_v ** 2) - (curr_v ** 2)) / (2 * dist_to_WZ)
                        while v[i - 1] > des_v:
                            a[i] = dec
                            x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i] * (tau ** 2))
                            v[i] = v[i - 1] + tau * a[i]
                            t[i] = t[i - 1] + tau
                            i += 1

                    i_when_v_is_des_v = i

                    # Whether our curr_v is above or below the RSWZ speed limit, we need to traverse the WZ.
                    des_x = x[i - 1] + dist_of_WZ
                    while x[i - 1] <= des_x:
                        a[i] = 0
                        x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i] * (tau ** 2))
                        v[i] = v[i - 1] + tau * a[i]
                        t[i] = t[i - 1] + tau
                        i += 1
                    end = i
                    self.comms.append((v2i[start], (start, end, i_when_v_is_des_v)))

                elif comm[0] == 'S':
                    curr_v = v[i - 1]
                    dist_to_WZ, stop_duration = int(comm[1]), int(int(comm[2]) / tau)
                    # To calculate the appropriate deceleration, we use the following kinematic equation
                    # a = (reduced_speed ** 2) - (speed ** 2) / ((2 * distance_to_WZ))
                    dec = -(curr_v ** 2) / (2 * dist_to_WZ)
                    while v[i - 1] > 0:
                        a[i] = dec
                        x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i] * (tau ** 2))
                        v[i] = v[i - 1] + tau * a[i]
                        if v[i] < 0:
                            v[i] = 0
                        t[i] = t[i - 1] + tau
                        i += 1

                    i_when_v_is_0 = i

                    # Now we iterate over the duration of the stop
                    for _ in range(stop_duration):
                        v[i] = v[i - 1]
                        x[i] = x[i - 1]
                        a[i] = 0
                        t[i] = t[i - 1] + tau
                        i += 1

                    end = i
                    self.comms.append((v2i[start], (start, end, i_when_v_is_0)))
            else:
                if counter % 20 == 0:
                    # 1st condition: is v[i] "far away" from v_max ?
                    if self.is_v_far_from_v_max(v[i - 1]):
                        # 2nd condition: is this our first iteration of the random trajectory phase?
                        if i == ran_t_start:
                            a[i] = self.acc_ran(v[i - 1], tau)
                        else:
                            # 3rd condition: what was our previous action?
                            if self.prev_action == self.acc_fast or self.prev_action == self.acc_slow:
                                # 4th condition: Were we accelerating fast or slow?
                                if self.prev_action == self.acc_fast:
                                    # 5th condition: Is our time step large? (Is our sampling rate big)
                                    if tau >= 1:
                                        a[i] = self.acc_ran(v[i - 1], tau, choice=4)
                                    else:
                                        a[i] = self.acc_ran(v[i - 1], tau, option=1)
                                else:
                                    a[i] = self.acc_ran(v[i - 1], tau, option=2)
                            elif self.prev_action == self.dec_fast or self.prev_action == self.dec_fast:
                                # 4th condition: Is v near zero?
                                if self.is_v_near_zero(v[i - 1]):
                                    # 5th condition: Were we decelerating fast or slow?
                                    if self.prev_action == self.dec_fast:
                                        # 6th condition: Is our time step large? (Is our sampling rate big)
                                        if tau >= 1:
                                            a[i] = self.acc_ran(v[i - 1], tau, choice=3)
                                        else:
                                            a[i] = self.acc_ran(v[i - 1], tau, option=3)
                                    else:
                                        a[i] = self.acc_ran(v[i - 1], tau, option=4)
                                else:
                                    a[i] = self.acc_ran(v[i - 1], tau)
                            elif self.prev_action == self.no_acc:
                                a[i] = self.acc_ran(v[i - 1], tau, option=5)
                    else:
                        # 2nd condition: is this our first iteration of the random trajectory phase?
                        if i == ran_t_start:
                            a[i] = self.acc_ran(v[i - 1], tau, option=6)
                        else:
                            # 3rd condition: What was the previous action?
                            if self.prev_action == self.acc_fast or self.prev_action == self.acc_slow:
                                # 4th condition: Were we accelerating fast or slow?
                                if self.prev_action == self.acc_fast:
                                    # 5th condition: Is v near v_max?
                                    if self.is_v_near_v_max(v[i - 1]):
                                        a[i] = self.acc_ran(v[i - 1], tau, option=6)
                                    else:
                                        # 6th condition: Is our time step large? (Is our sampling rate big)
                                        if tau >= 1:
                                            a[i] = self.acc_ran(v[i - 1], tau, option=7)
                                        else:
                                            a[i] = self.acc_ran(v[i - 1], tau, option=8)
                                else:
                                    # 5th condition: Is v near v_max?
                                    if self.is_v_near_v_max(v[i - 1]):
                                        a[i] = self.acc_ran(v[i - 1], tau, choice=3)
                                    else:
                                        # 6th condition: Is our time step large? (Is our sampling rate big)
                                        if tau >= 1:
                                            a[i] = self.acc_ran(v[i - 1], tau, option=3)
                                        else:
                                            a[i] = self.acc_ran(v[i - 1], tau)
                            elif self.prev_action == self.dec_fast or self.prev_action == self.dec_slow:
                                # 4th condition: Were we decelerating fast or slow?
                                if self.prev_action == self.dec_fast:
                                    # 5th condition: Is our time step large? (Is our sampling rate big)
                                    if tau >= 1:
                                        a[i] = self.acc_ran(v[i - 1], tau, option=7)
                                    else:
                                        a[i] = self.acc_ran(v[i - 1], tau, choice=4)
                                else:
                                    a[i] = self.acc_ran(v[i - 1], tau, option=7)
                            elif self.prev_action == self.no_acc:
                                # 4th condition: is v near v max?
                                if self.is_v_near_v_max(v[i - 1]):
                                    a[i] = self.acc_ran(v[i - 1], tau, option=6)
                                else:
                                    a[i] = self.acc_ran(v[i - 1], tau)
                else:
                    a[i] = a[i - 1]
                # Update trajectory information
                x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i] * (tau ** 2))
                v[i] = v[i - 1] + tau * a[i]
                # If our updated velocity is less than 0, we set it to 0 to represent a stop.
                v[i] = v[i] if v[i] > 0 else 0
                t[i] = t[i - 1] + tau

                counter += 1
                i += 1

        # Given the current velocity we're traveling and the time remaining from the trip, we need to calculate
        # the value of acceleration to decelerate our current velocity such that it reaches 0 at the end.
        # We will be decelerating the last 10th of the trip
        # DECELERATION PHASE:
        dec_t_start = ran_t_end
        dec_t_end = len(t) - 1
        # Calculate appropriate deceleration value such that we make our vehicle come to a stop.
        dec_duration = (dec_t_end - dec_t_start + 1) * tau
        dec = - (v[dec_t_start - 1] / dec_duration)
        for i in range(dec_t_start, dec_t_end + 1):
            a[i] = dec
            x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i] * (tau ** 2))
            v[i] = v[i - 1] + tau * a[i]
            t[i] = t[i - 1] + tau

        a[dec_t_end] = 0
        v[dec_t_end] = 0

        self.cache = dict({'t': t, 'x': x, 'v': v, 'a': a})

    def acc_acc(self, choice, v_init, duration, acc_duration):
        """
        A HoF that will return the acceleration scenario for the CAV during the acceleration phase. With the given
        acceleration function, the CAV will accelerate up to the desired velocity (a value between init_v + 5 and
        the max velocity).

        :param choice: Which acceleration scenario to choose (int).
        :param v_init: The initial velocity of CAV, in m/s. (np.float64).
        :param duration: The duration of acceleration phase, in seconds (int).
        :param acc_duration: The duration of acceleration phase represented discretely in time steps (float).
        :return: A function representing the acceleration scenario for this trip's acceleration phase.
        """

        v_des = int(r.uniform(v_init + 5, self.v_max))  # v_des = [init_v + 5, max_v]
        v_fast = int(r.uniform((3 * v_des) / 4, (7 * v_des) / 8))  # v_fast = [(3/4) * v_des, (7/8) * v_des]

        def acc_quickly_then_slowly(curr_t):
            """
            A function that will simulate the acceleration of a car speeding up quickly then speeding up slowly. First,
            the car will speed up quickly for half the duration then speed up slowly for the rest of the duration.

            :param curr_t: The time within the acceleration phase (int).
            :return: The new acceleration of the vehicle.
            """
            if curr_t <= acc_duration / 2:
                return a_fast
            else:
                return a_slow

        def acc_slowly_then_quickly(curr_t):
            """
            A function that will simulate the acceleration of a car speeding up slowly then speeding up quickly. First,
            the car will speed up slowly for half of the duration then speed up quickly for the rest of the duration.

            :param curr_t: The time within the acceleration phase (int).
            :return: The new acceleration of the vehicle.
            """
            if curr_t <= acc_duration / 2:
                return a_slow
            else:
                return a_fast

        def acc_quickly(curr_t):
            """
            A function that will simulate the acceleration of a car speeding up quickly. The car will be speeding up
            quickly until it reaches v_des then remains in constant velocity in the acceleration phase.

            :param curr_t: The time within the acceleration phase (int).
            :return: The new acceleration of the vehicle.
            """
            if curr_t > acc_duration / 2:
                return 0
            else:
                return a_fast

        def acc_slowly(curr_t=None):
            """
            A function that will simulate the acceleration of a car speeding up slowly. The car will be speeding up
            slowly for the entire duration of the acceleration phase.

            :param curr_t: The time within the acceleration phase (int).
            :return: The new acceleration of the vehicle.
            """
            return a_slow

        if choice == 1:
            a_fast = (v_fast - v_init) / (duration / 2)
            a_slow = (v_des - v_fast) / (duration / 2)
            return acc_quickly_then_slowly
        elif choice == 2:
            a_fast = (v_fast - v_init) / (duration / 2)
            a_slow = (v_des - v_fast) / (duration / 2)
            return acc_slowly_then_quickly
        elif choice == 3:
            a_fast = (v_des - v_init) / (duration / 2)
            return acc_quickly
        elif choice == 4:
            a_slow = (v_des - v_init) / duration
            return acc_slowly

    # Control methods to influence the updated acceleration of the CAV during the random trajectory phase.

    def is_v_far_from_v_max(self, v):
        """
        A function that checks whether the current velocity is "far from" the maximum velocity. We consider v to be
        far away from v_max if v is within 1/3rd of v_max. NOTE: This set value of 1/3 can be tweaked (we could also
        say 1/6).

        :param v: The previous velocity of the vehicle, in m/s (np.float64).
        :return: A bool indicating whether v is far from v_max.
        """
        return v <= int((1 / 4) * self.v_max)

    def is_v_near_zero(self, v):
        """
        A function that checks whether the current velocity is "near" zero. We consider v to be near zero if v is within
        5 m/s of 0 m/s. NOTE: This set value of 5 can be tweaked (we could also say 10 m/s).

        :param v: The previous velocity of the vehicle, in m/s (np.float64).
        :return: A bool indicating whether v is near 0.
        """
        return v <= 5

    def is_v_near_v_max(self, v: np.float64):
        """
        A function that checks whether the current velocity is "near" v_max. We consider v to be near v_max if v is
        within 5 m/s of v_max. NOTE: This set value of 5 can be tweaked (we could also say 10 m/s).

        :param v: The previous velocity of the vehicle, in m/s (np.float64).
        :return: A bool indicating whether v is near v_max.
        """
        return (v - self.v_max) <= 10

    def acc_ran(self, v, tau, option=0, choice=None):
        """
        A function that serves as the control panel for acceleration during the random trajectory phase. Returns the
        updated acceleration of the CAV based on several parameters.

        :param v: The previous velocity of the vehicle, in m/s (np.float64).
        :param tau: The timestep of the current trip (int).
        :param option: An option that determines which pool of actions we will randomly select from (int).
        :param choice: An optional parameter such that we restrict ourselves to a single action (int).
        :return: The new acceleration of the vehicle.
        """

        all_actions = [self.acc_fast, self.acc_slow, self.dec_fast, self.dec_slow, self.no_acc]
        subset_actions = []

        if choice is not None:  # We force a particular action
            if choice == 0:
                return self.acc_fast()
            elif choice == 1:
                return self.acc_slow()
            elif choice == 2:
                return self.dec_fast(v, tau)
            elif choice == 3:
                return self.dec_slow(v, tau)
            else:
                return self.no_acc()

        # Option X means we select ANY random action from ...

        if option == 0:  # all_actions
            subset_actions = all_actions
        elif option == 1:  # (acc fast, no acc)
            subset_actions = [self.acc_fast, self.no_acc]
        elif option == 2:  # (acc slow, no acc)
            subset_actions = [self.acc_slow, self.no_acc]
        elif option == 3:  # (dec slow, no acc)
            subset_actions = [self.dec_slow, self.no_acc]
        elif option == 4:  # (dec slow, no acc, acc fast, acc slow)
            subset_actions = [self.dec_slow, self.no_acc, self.acc_fast, self.acc_fast]
        elif option == 5:  # (no acc, acc fast, acc slow)
            subset_actions = [self.no_acc, self.acc_fast, self.acc_slow]
        elif option == 6:  # (dec fast, dec slow)
            subset_actions = [self.dec_fast, self.dec_slow]
        elif option == 7:  # (dec fast, dec slow, no acc)
            subset_actions = [self.dec_fast, self.dec_slow, self.no_acc]
        elif option == 8:  # (acc fast, acc slow, no acc)
            subset_actions = [self.acc_fast, self.acc_slow, self.no_acc]

        # Randomly choose action from subset_actions
        action = r.choice(subset_actions)

        if action == self.acc_fast or action == self.acc_slow or action == self.no_acc:
            return action()
        return action(v, tau)

    # Control panel for accelerating and decelerating during the random trajectory phase.

    def acc_fast(self):
        """
        An action that represents the vehicle accelerating quickly a rate of 2.25 m/s^2.

        :return: The new acceleration of the vehicle.
        """
        self.prev_action = self.acc_fast
        return 2

    def acc_slow(self):
        """
        An action that represents the vehicle accelerating slowly at rate of 1 m/s^2.

        :return: The new acceleration of the vehicle.
        """
        self.prev_action = self.acc_fast
        return 1

    def dec_fast(self, v, tau):
        """
        An action that represents the vehicle decelerating quickly a rate of -3 m/s^2.

        :param v: The previous velocity of the vehicle, in m/s (np.float64).
        :param tau: The timestep of the current trip (int).
        :return: The new acceleration of the vehicle.
        """
        # Check whether v will go below zero, If it does, do not accelerate/decelerate.
        # v[i - 1] + tau * candidate_a = v[i]
        if (v + tau * -3) < 0:
            self.prev_action = self.no_acc
            return self.no_acc()
        self.prev_action = self.dec_fast
        return -3

    def dec_slow(self, v, tau):
        """
        An action that represents the vehicle decelerating slowly at a rate of -1 m/s^2.

        :param v: The previous velocity of the vehicle, in m/s (np.float64).
        :param tau: The timestep of the current trip (int).
        :return: The new acceleration of the vehicle.
        """
        # Check whether v will go below zero. If it does, do not accelerate/decelerate.
        # v[i - 1] + tau * candidate_a = v[i]
        if (v + tau * -1) < 0:
            self.prev_action = self.no_acc
            return self.no_acc()
        self.prev_action = self.dec_fast
        return -1

    def no_acc(self):
        """
        An action that represents the vehicle not accelerating.

        :return: The new acceleration of the vehicle.
        """
        self.prev_action = self.no_acc
        return 0

    # HELPER METHODS

    def report(self):
        """
        Returns a DataFrame with information about the CAV's most recent trip.

        :return: A pandas DataFrame with information on the CAV's position, velocity, and acceleration.
        """
        assert self.cache is not None, "Cannot print report as cache is empty."

        data = np.column_stack((self.cache['t'], self.cache['x'], self.cache['v'], self.cache['a']))
        dataframe = pd.DataFrame(data, columns=['time', 'position', 'velocity', 'acceleration'])
        return dataframe
