import numpy as np
import pandas as pd
import random as r

"""
Eventually, we'll have control over the trajectory of the vehicle based on work zone reduction speeds and stop signs. We
will use a "mode" to refer to what the car should be doing. For example, we could have the car in "drive regular" mode
then when there's a rswz we'd change the mode to "rswz" until some condition has been reached.
"""


class Vehicle:
    """
    An object that represents a Connected and Autonomous Vehicle (CAV). We can generate trajectories of the vehicle
    based on various parameters.
    """

    def __init__(self, v_max, a_max, model, make):
        self.v_max = v_max
        self.a_max = a_max
        self.model = model
        self.make = make
        self.cache = None
        self.prev_action = None
        self.actions = []

    def trajectory(self, v_init, timestep, duration, seed):
        """
        Reports the velocity, position, and acceleration of the CAV at each timestep up until duration as a dictionary.

        :param v_init:
        :param timestep:
        :param duration:
        :param seed:
        """
        # Clear cache from previous trip
        self.prev_action = None
        self.cache = None

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
        acc = self.acc(int(r.uniform(0, 4)), v_init, acc_duration, acc_t_end - 1)
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

        for i in range(ran_t_start, ran_t_end):
            # Parse input
            comm = input("").split(",")

            if comm[0] == 'rs':

                # comm = ['rs', 'dist_to_WZ', 'speed_limit_in_WZ', 'distance/duration']

                speed = v[i - 1]
                distance_to_WZ = comm[1]
                reduced_speed = comm[2]
                distance_of_WZ = comm[3]

                if speed <= reduced_speed:
                    a[i] = 0
                else:

                    a[i] = deceleration

                # if mode == 'rswz':
                # execute whatever we need
                # elif mode == 'stop':
                # execute wahtever we need
                # ...
                # ...
                # once we're dne, we set mode == "random"
                # else:
                # rs
                # dist_to_WZ
                # speed_limit_in_WZ, dist / duration
                # rs, 100, 20, 500
                #
                # stop, dist_to_WZ, duration_of_stop
                # stop, 200, 120
                comm = ['']
            elif comm[0] == 'stop':

                # comm = ['stop', 'dist_to_WZ', 'duration_of_stop']

                comm = ['']
            else:
                if counter % 20 == 0:
                    # 1st condition: is v[i] "far away" from v_max ?
                    if self.is_v_far_from_v_max(v[i - 1]):
                        # 2nd condition: is this our first iteration of the random trajectory phase?
                        if i == ran_t_start:
                            a[i] = self.control(v[i - 1], a[i - 1], tau)
                        else:
                            # 3rd condition: what was our previous action?
                            if self.prev_action == self.acc_fast or self.prev_action == self.acc_slow:
                                # 4th condition: Were we accelerating fast or slow?
                                if self.prev_action == self.acc_fast:
                                    # 5th condition: Is our time step large? (Is our sampling rate big)
                                    if tau >= 1:
                                        a[i] = self.control(v[i - 1], a[i - 1], tau, choice=4)
                                    else:
                                        a[i] = self.control(v[i - 1], a[i - 1], tau, option=1)
                                else:
                                    a[i] = self.control(v, a, tau, option=2)
                            elif self.prev_action == self.dec_fast or self.prev_action == self.dec_fast:
                                # 4th condition: Is v near zero?
                                if self.is_v_near_zero(v[i - 1]):
                                    # 5th condition: Were we decelerating fast or slow?
                                    if self.prev_action == self.dec_fast:
                                        # 6th condition: Is our time step large? (Is our sampling rate big)
                                        if tau >= 1:
                                            a[i] = self.control(v[i - 1], a[i - 1], tau, choice=3)
                                        else:
                                            a[i] = self.control(v[i - 1], a[i - 1], tau, option=3)
                                    else:
                                        a[i] = self.control(v[i - 1], a[i - 1], tau, option=4)
                                else:
                                    a[i] = self.control(v[i - 1], a[i - 1], tau)
                            elif self.prev_action == self.no_acc:
                                a[i] = self.control(v[i - 1], a[i - 1], tau, option=5)
                    else:
                        # 2nd condition: is this our first iteration of the random trajectory phase?
                        if i == ran_t_start:
                            a[i] = self.control(v[i - 1], a[i - 1], tau, option=6)
                        else:
                            # 3rd condition: What was the previous action?
                            if self.prev_action == self.acc_fast or self.prev_action == self.acc_slow:
                                # 4th condition: Were we accelerating fast or slow?
                                if self.prev_action == self.acc_fast:
                                    # 5th condition: Is v near v_max?
                                    if self.is_v_near_v_max(v[i - 1]):
                                        a[i] = self.control(v[i - 1], a[i - 1], tau, option=6)
                                    else:
                                        # 6th condition: Is our time step large? (Is our sampling rate big)
                                        if tau >= 1:
                                            a[i] = self.control(v[i - 1], a[i - 1], tau, option=7)
                                        else:
                                            a[i] = self.control(v[i - 1], a[i - 1], tau, option=8)
                                else:
                                    # 5th condition: Is v near v_max?
                                    if self.is_v_near_v_max(v[i - 1]):
                                        a[i] = self.control(v[i - 1], a[i - 1], tau, choice=3)
                                    else:
                                        # 6th condition: Is our time step large? (Is our sampling rate big)
                                        if tau >= 1:
                                            a[i] = self.control(v[i - 1], a[i - 1], tau, option=3)
                                        else:
                                            a[i] = self.control(v[i - 1], a[i - 1], tau)
                            elif self.prev_action == self.dec_fast or self.prev_action == self.dec_slow:
                                # 4th condition: Were we decelerating fast or slow?
                                if self.prev_action == self.dec_fast:
                                    # 5th condition: Is our time step large? (Is our sampling rate big)
                                    if tau >= 1:
                                        a[i] = self.control(v[i - 1], a[i - 1], tau, option=7)
                                    else:
                                        a[i] = self.control(v[i - 1], a[i - 1], tau, choice=4)
                                else:
                                    a[i] = self.control(v[i - 1], a[i - 1], tau, option=7)
                            elif self.prev_action == self.no_acc:
                                # 4th condition: is v near v max?
                                if self.is_v_near_v_max(v[i - 1]):
                                    a[i] = self.control(v[i - 1], a[i - 1], tau, option=6)
                                else:
                                    a[i] = self.control(v[i - 1], a[i - 1], tau)
            # Update trajectory information
            counter += 1
            x[i] = x[i - 1] + tau * v[i - 1] + (0.5 * a[i] * (tau ** 2))
            v[i] = v[i - 1] + tau * a[i]
            t[i] = t[i - 1] + tau

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

    def acc(self, choice, v_init, duration, acc_duration):
        """
        A HoF that will return the acceleration scenario for the CAV during the acceleration phase. With the given
        acceleration function, the CAV will accelerate up to the desired velocity (a value between init_v + 5 and
        the max velocity)

        :param choice: which acceleration scenario to choose (int)
        :param v_init: initial velocity of CAV (float)
        :param duration: duration of acceleration phase in seconds (int)
        :param acc_duration: duration of acceleration phase represented discretely in time steps (int)
        :return: function
        """

        v_des = int(r.uniform(v_init + 5, self.v_max))  # v_des = [init_v + 5, max_v]
        v_fast = int(r.uniform((3 * v_des) / 4, (7 * v_des) / 8))  # v_fast = [(3/4) * v_des, (7/8) * v_des]

        def acc_quickly_then_slowly(curr_t):
            """
            A function that will simulate the acceleration of a car speeding up quickly then speeding up slowly. First,
            the car will speed up quickly for half the duration then speed up slowly for the rest of the duration.

            :param curr_t: current time within the acceleration phase (int)
            :return: float
            """
            if curr_t <= acc_duration / 2:
                return a_fast
            else:
                return a_slow

        def acc_slowly_then_quickly(curr_t):
            """
            A function that will simulate the acceleration of a car speeding up slowly then speeding up quickly. First,
            the car will speed up slowly for half of the duration then speed up quickly for the rest of the duration.

            :param curr_t: current time within the acceleration phase (int)
            :return: float
            """
            if curr_t <= acc_duration / 2:
                return a_slow
            else:
                return a_fast

        def acc_quickly(curr_t):
            """
            A function that will simulate the acceleration of a car speeding up quickly. The car will be speeding up
            quickly until it reaches v_des then remains in constant velocity in the acceleration phase.

            :param curr_t: current time within the acceleration phase (int)
            :return: float
            """
            if curr_t > acc_duration / 2:
                return 0
            else:
                return a_fast

        def acc_slowly(curr_t=None):
            """
            A function that will simulate the acceleration of a car speeding up slowly. The car will be speeding up
            slowly for the entire duration of the acceleration phase.

            :param curr_t: current time within the acceleration phase (int)
            :return: float
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

    def is_v_far_from_v_max(self, v):
        """
        A function that checks whether the current velocity is "far from" the maximum velocity. We consider v to be
        far away from v_max if v is within 1/3rd of v_max. NOTE: This set value of 1/3 can be tweaked (we could also
        say 1/6).
        =
        :param v: current velocity of CAV (int)
        :return: boolean
        """
        return v <= int((1 / 4) * self.v_max)

    def is_v_near_zero(self, v):
        """
        A function that checks whether the current velocity is "near" zero. We consider v to be near zero if v is within
        5 m/s of 0 m/s. NOTE: This set value of 5 can be tweaked (we could also say 10 m/s)

        :param v: current velocity of CAV (int)
        :return: boolean
        """
        return v <= 5

    def is_v_near_v_max(self, v):
        """
        A function that checks whether the current velocity is "near" v_max. We consider v to be near v_max if v is
        within 5 m/s of v_max. NOTE: This set value of 5 can be tweaked (we could also say 10 m/s)

        :param v: current velocity of CAV (int)
        :return: boolean
        """
        return (v - self.v_max) <= 10

    def control(self, v, a, tau, option=0, choice=None):
        """
        A HoF that will control the trajectory of the CAV based on various parameters.

        :param v: current velocity of the CAV (float)
        :param a: current acceleration of the CAV (float)
        :param tau: time step of the current trajectory (int)
        :param option: which pool of actions we will randomly select from (int)
        :param choice: optional parameter such that we restrict ourselves to a single action. (int)
        :return: float
        """
        all_actions = [self.acc_fast, self.acc_slow, self.dec_fast, self.dec_slow, self.no_acc]
        subset_actions = []

        if choice:  # We force a particular action
            return all_actions[choice](v, a, tau)

        # Option X means we select ANY random action from ...

        if option == 0:  # all_actions
            return r.choice(all_actions)(v, a, tau)
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

        return r.choice(subset_actions)(v, a, tau)

    # TODO: To add more complexity to our acceleration/deceleration, we can take into account the max velocity and
    #  make a reasonable estimate of the acceleration/deceleration.

    # IMPORTANT
    # TODO: Fix these functions such that we don't exceed a_max NOR exceed v_max NOR v go below 0!!!!!!
    def acc_fast(self, v, a, tau):
        # Check whether a will exceed a_max or not
        self.prev_action = self.acc_fast
        return 2.25

    def acc_slow(self, v, a, tau):
        # Check whether a will exceed a_max or not
        self.prev_action = self.acc_fast
        return 1

    def dec_fast(self, v, a, tau):
        # Check whether v will go below zero.
        # v[i - 1] + tau * a[i] = v[i]
        if (v + tau * -3) < 0:
            self.prev_action = self.no_acc
            return 0
        self.prev_action = self.dec_fast
        return -3

    def dec_slow(self, v, a, tau):
        if (v + tau * -1) < 0:
            self.prev_action = self.no_acc
            return 0
        self.prev_action = self.dec_fast
        return -1

    def no_acc(self, v, a, tau):
        self.prev_action = self.no_acc
        return 0

    def report(self):
        """
        A function that returns a dataframe containing all the information of the CAV's most recent trip.

        :return: pandas dataframe
        """
        assert self.cache is not None, "Cannot print report as cache is empty."

        data = np.column_stack((self.cache['t'], self.cache['x'], self.cache['v'], self.cache['a']))
        dataframe = pd.DataFrame(data, columns=['time', 'position', 'velocity', 'acceleration'])
        return dataframe
