import random as r
import numpy as np


# Fastest acceleration is typically 10 m/s^2
# Fastest deceleration is typically -20 m/s^2
# TODO: Implement a way to set the acceleration s.t. velocity reaches 0 (simulating a stop)
# TODO: Perhaps break down the total_duration into certain ranges arbitrarily.
class Acceleration:
    def __init__(self, seed, max_a, max_v):
        self.max_a = max_a
        self.max_v = max_v
        self.prev_action = None  # Keep track of our previous acceleration method
        r.seed(seed)

    def set_acceleration(self, total_duration, t, a, v, x):
        # The main method that will really change the acceleration.
        # We will use the t, a, v, and x, and total_duration to determine what method we will execute when we are
        # at certain boundaries (v -> 0 or v -> max_v, a -> max_a, a -> 0, t -> total_duration,

        # BOUNDARY CONDITIONS: (Will be hard coded in some way)
        # 1. v near max_v [Assume within 20 m/s of the max_v, we should reduce the speed SLOWLY]
        # 2. v near max_v [Assume within 10 m/s of the max_v, we should reduce the speed QUICKER] a.
        # 3. v near max_v [Assume within 5 m/s of the max_v, we should reduce the speed QUICKLY] a.
        # 4. ARBITRARILY (depend on t and total_duration)
        #   4a. v near 0 [reduce speed SLOWLY until 0]
        #   4b. v near 0 [increase speed QUICKLY]
        #   4c. v near 0 [increase speed SLOWLY]
        #   4d. v near 0 [reduce speed QUICKLY]
        # 5. t near total_duration [Within the tenth of the total_duration]
        #   - Reduce speed until v = 0 and t = total_duration.
        #   [keep prev_action to decelerate_quickly or decelerate_slowly]
        # 6. a near max_a [Assume within a range of 5 m/s^2, reduce acceleration QUICKER] a.
        # 7. a near max_a [Assume within a range of 10 m/s^2 reduce acceleration SLOWLY]
        # 8. ARBITRARILY (depend on t and total_duration)
        #   8a. a near 0 [reduce acceleration SLOWLY]
        #   8b. a near 0 [reduce acceleration QUICKLY]
        #   8c. a near 0 [reduce acceleration QUICKER]
        #   8d. a near 0 [reduce acceleration QUICKLY if t near total_duration [get v to 0]

        # PSEUDORANDOM IF NOT NEAR ANY BOUNDARY CONDITIONS:

        if self.max_v - v <= 10:  # Near the max velocity, should reduce quickly.
            action = self.choose_acceleration_method(3)
            if self.max_v <= 5:
                action = self.choose_acceleration_method(3)
                a = action(total_duration, t, a, v)


        return a

    def choose_acceleration_method(self, option):
        if option == 1:
            return self.accelerate_quickly
        elif option == 2:
            return self.accelerate_slowly
        elif option == 3:
            return self.decelerate_quickly
        elif option == 4:
            return self.decelerate_slowly
        elif option == 5:
            return 0  # Don't accelerate nor decelerate.

    # Our acceleration methods need to update accordingly and relative to the current acceleration and other
    # information

    def accelerate_quickly(self, total_duration, t, a, v):
        return 0

    def accelerate_slowly(self, total_duration, t, a, v):
        return 1

    def decelerate_quickly(self, total_duration, t, a, v):
        if np.isclose([t / total_duration, 1], rtol=0.05, atol=0):
            return -a
        # TODO: Add complexity to a fast deceleration based on the current
        #  velocity and time beside the time near the end of the total duration.
        return -a * 0.75

    def decelerate_slowly(self, total_duration, t, a, v):
        return 2
