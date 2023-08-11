import random
import format
from vehicle.vehicle import *


class Attack:
    """
    An object that represents the various attack scenarios that can come about from V2I/V2X communication.
    """

    def __init__(self, v_max, a_max):
        """
        The constructor for the Attack module.

        :param v_max: The maximum velocity of the vehicle, in m/s (int).
        :param a_max: The maximum acceleration of the vehicle, in m/s^2 (int).
        """
        self.vehicle = Vehicle(v_max, a_max)
        self.v2i_comms = ["S,100,20", "RS,100,10,500"]
        self.attack_panel = \
            {
                0: self.eq,
                1: self.ignore_stop,
                2: self.ignore_rs,
                3: self.swz_rs,
                4: self.dwz_rs,
                5: self.lwz_rs,
                6: self.rswz,
                7: self.dwz_stop,
                8: self.dur_wz_stop,
                9: self.stop
            }

    def traj(self, v_init, timestep, duration, seed):
        """
        Return a benign trajectory of the vehicle.

        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory (int).
        :return: A benign trajectory of the object's vehicle.
        """
        self.vehicle.trajectory(v_init, timestep, duration, self.v2i_comms, seed=seed)
        return self.vehicle.report()

    def compare(self, v_init, timestep, duration, seed, scenario=1):
        """
        A function that compares the benign trajectory with a faulty trajectory that was under a specific attack
        scenario by displaying and visualizing the differences between both trajectories.

        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory (int).
        :param scenario: Which attack scenario to execute (int).
        """
        assert scenario in self.attack_panel, "Choose a valid attack mechanism ranging from 0-9"

        random.seed(seed)
        benign_traj = self.traj(v_init, timestep, duration, seed=seed)
        faulty_traj = None

        if scenario == 0:
            faulty_traj = self.eq(benign_traj)
        elif scenario == 1:
            faulty_traj = self.ignore_stop(benign_traj, v_init, timestep, duration, seed)
        elif scenario == 2:
            faulty_traj = self.ignore_rs(benign_traj)
        elif scenario == 3:
            perturbed_v = int(input("Input perturbed reduced speed in work zone: "))
            faulty_traj = self.swz_rs(benign_traj, v_init, perturbed_v, timestep, duration, seed)
        elif scenario == 4:
            perturbed_dist = int(input("Input perturbed distance to work zone: "))
            faulty_traj = self.dwz_rs(benign_traj, v_init, perturbed_dist, timestep, duration, seed)
        elif scenario == 5:
            perturbed_len = int(input("Input perturbed length of work zone: "))
            faulty_traj = self.lwz_rs(benign_traj, v_init, perturbed_len, timestep, duration, seed)
        elif scenario == 6:
            perturbed = input(
                "Input perturbed distance to, reduced speed of, and length of work zone as csv in that order: ").split(
                ",")
            perturbed = list(map(int, perturbed))
            faulty_traj = self.rswz(benign_traj, v_init, perturbed, timestep, duration, seed)
        elif scenario == 7:
            perturbed_dist = int(input("Input perturbed distance to work zone: "))
            faulty_traj = self.dwz_stop(benign_traj, perturbed_dist)
        elif scenario == 8:
            perturbed_dur = int(input("Input perturbed duration of work zone: "))
            faulty_traj = self.dur_wz_stop(benign_traj, v_init, perturbed_dur, timestep, duration, seed)
        elif scenario == 9:
            perturbed = input("Input perturbed distance to and duration of stop at work zone as csv in that order: ") \
                .split(",")
            perturbed = list(map(int, perturbed))
            faulty_traj = self.stop(benign_traj, v_init, perturbed, timestep, duration, seed)

        format.plot_trajectory_compare(faulty_traj, benign_traj)

    # Attack panel

    def ignore_stop(self, truth, v_init, timestep, duration, seed):
        """
        Return a perturbed trajectory where a V2I s communication is ignored which ultimately results in a crash.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory. (int)
        :return: The perturbed trajectory.
        """

        # Get a random S comm randomly within v2i_comms
        _, stop_idx, window, _, _ = self.get_random_S_info()

        return self.simulate_crash_s(truth, window, stop_idx, v_init, timestep, duration, seed)

    def ignore_rs(self, truth, v_init, timestep, duration, seed):
        """
        Return a perturbed trajectory where a V2I rs communication is ignored which ultimately results in a crash.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory. (int)
        :return: The perturbed trajectory.
        """

        # Get a random RS comm randomly within v2i_comms
        _, rs_idx, window, _, _, _ = self.get_random_RS_info()

        return self.simulate_crash_rs(truth, window, rs_idx, v_init, timestep, duration, seed)

    def swz_rs(self, truth, v_init, perturbed_v, timestep, duration, seed):
        """
        Return a perturbed trajectory where the reduced speed in the work zone is perturbed.
        We have a few cases of what the perturbed trajectory will look like:

        1. Crash/no crash because the perturbed reduced speed in the work zone may or may not cause a crash thus we
           randomize the outcome using pseudo-randomness.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param perturbed_v: The perturbed reduced speed in the work zone (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory. (int)
        :return: The perturbed trajectory.
        """

        outcome = random.choice([0, 1])
        rs_comm, rs_idx, window, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        if outcome == 0:
            # (1): Find one RS comm and perturb it. Everything other comm stays the same.
            # Create our perturbed v2i_comms
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(dist_to_WZ, perturbed_v, len_of_WZ)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty
        elif outcome == 1:
            # (2): Crash
            # If our perturbed v is less than the actual reduce speed in the work zone, no crash should happen.
            if rs_comm.split(",")[2] >= perturbed_v:
                return self.eq(truth)
            else:
                return self.simulate_crash_rs(truth, window, rs_idx, v_init, timestep, duration, seed)

    def dwz_rs(self, truth, v_init, perturbed_dist, timestep, duration, seed):
        """
        Return a perturbed trajectory where the distance to the work zone is perturbed.
        We have a few cases of what the perturbed trajectory will look like:

        1. Crash because the actual distance is smaller than the perturbed distance, so we're not reduced our speed prior
           to entering the work zone.
        2. Crash because thea actual distance to the work zone is near the length of the work zone plus the perturbed
           distance to the work zone which implies our vehicle, based on the malicious V2I communication, will think it
           has traversed the work zone before even entering the actual work zone.
        3. The actual distance to the work zone is larger than the perturbed distance and this
           may or may not cause a crash thus we randomize the outcome.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param perturbed_dist: The perturbed distance to the work zone (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory. (int)
        :return: The perturbed trajectory.
        """

        outcome = random.choice([0, 1])
        rs_comm, rs_idx, window, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        if dist_to_WZ <= perturbed_dist or abs(dist_to_WZ - perturbed_dist - len_of_WZ) <= 10 or outcome == 0:
            return self.simulate_crash_rs(truth, window, rs_idx, v_init, timestep, duration, seed)
        else:
            # No crash but our trajectory is perturbed
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(perturbed_dist, reduced_speed, len_of_WZ)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def lwz_rs(self, truth, v_init, perturbed_len, timestep, duration, seed):
        """
        Return a perturbed trajectory where the distance/length of the work zone is perturbed.
        We have a few cases of what the perturbed trajectory will look like:

        1. Crash because the perturbed distance/length is less than the actual distance/length.
        2. No crash because the perturbed distance/length is greater than the actual distance/length.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param perturbed_len: The perturbed distance/length of the work zone (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory (int).
        :return: The perturbed trajectory.
        """

        rs_comm, rs_idx, window, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        if len_of_WZ > perturbed_len:
            return self.simulate_crash_rs(truth, window, rs_idx, v_init, timestep, duration, seed)
        else:
            # No crash but our trajectory is perturbed
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(dist_to_WZ, reduced_speed, perturbed_len)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def rswz(self, truth, v_init, perturbed, timestep, duration, seed):
        """
        Return a perturbed trajectory where the distance to, reduced speed in, and distance/length of the work zone is
        perturbed. We have a few cases of what the perturbed trajectory will look like:

        1. Crash because the perturbed V2I communication is incorrect in every way.
        2. No crash by chance.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param perturbed: The perturbed communication (list).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int). 
        :param seed: The seed of the benign trajectory (int).
        :return: The perturbed trajectory.
        """
        outcome = random.choice([0, 1])
        rs_comm, rs_idx, window, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        if outcome == 0:
            return self.simulate_crash_rs(truth, window, rs_idx, v_init, timestep, duration, seed)
        else:
            # Unpack perturbed values
            perturbed_dist, perturbed_v, perturbed_len = perturbed
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(perturbed_dist, perturbed_v, perturbed_len)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def dwz_stop(self, truth, v_init, perturbed_dist, timestep, duration, seed):
        """
        Return a perturbed trajectory where the distance to the work zone is perturbed. We have a few cases of what
        the perturbed trajectory will look like:

        1. Crash because we stop before the actual stop of the work zone, and we move into the actual work zone as a
           result.
        2. No crash because the difference between the distances to the work zone is trivial.
        3. Crash because the difference between the distances to the work zone is non-trivial.


        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param perturbed_dist: The perturbed distance to the work zone, in meters (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory (int).
        :return: The perturbed trajectory.
        """
        _, stop_idx, window, dist_to_WZ, _ = self.get_random_S_info()

        if 0 < abs(dist_to_WZ - perturbed_dist) < 5:
            return self.eq(truth)
        else:
            return self.simulate_crash_s(truth, window, stop_idx, v_init, timestep, duration, seed)

    def dur_wz_stop(self, truth, v_init, perturbed_dur, timestep, duration, seed):
        """
        Return a perturbed trajectory where the duration of the stop at the work zone is perturbed.
        We have a few cases of what the perturbed trajectory will look like:

        1. Crash because the perturbed duration of the stop less than the actual duration of the stop.
        2. The perturbed duration of the stop is greater than the actual duration of the stop and 
        that may or may not cause a crash, so we decide the outcome randomly.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param perturbed_dur: The perturbed duration of the stop at the work zone, in seconds (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int). 
        :param seed: The seed of the benign trajectory (int).
        :return: The perturbed trajectory.
        """
        outcome = random.choice([0, 1])
        stop_comm, stop_idx, window, dist_to_WZ, dur_of_WZ = self.get_random_S_info()
        if dur_of_WZ < perturbed_dur or perturbed_dur > dur_of_WZ and outcome == 0:
            return self.simulate_crash_s(truth, window, stop_idx, v_init, timestep, duration, seed)
        else:
            self.v2i_comms[stop_idx] = self.perturb_s_comm(dist_to_WZ, perturbed_dur)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def stop(self, truth, v_init, perturbed, timestep, duration, seed):
        """
        Return a perturbed trajectory where the distance to and duration of the stop at the work zone is perturbed.
        We have a few cases of what the perturbed trajectory will look like:

        1. Crash because the perturbed V2I communication is incorrect in every way.
        2. No crash by chance.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param perturbed: The perturbed communications (list).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int). 
        :param seed: The seed of the benign trajectory. (int)
        :return: The perturbed trajectory.
        """

        outcome = random.choice([0, 1])
        stop_comm, stop_idx, window, dist_to_WZ, dur_of_WZ = self.get_random_S_info()

        if outcome == 0:
            return self.simulate_crash_s(truth, window, stop_idx, v_init, timestep, duration, seed)
        else:
            # Unpack perturbed values
            perturbed_dist, perturbed_dur = perturbed
            self.v2i_comms[stop_idx] = self.perturb_s_comm(perturbed_dist, perturbed_dur)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def eq(self, truth):
        """
        Return a copy of the truth trajectory as the faulty trajectory ends up becoming benign.
        
        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :return: Copy of the benign trajectory.
        """

        return truth.copy()

    # HELPER METHODS

    def simulate_crash_s(self, truth, window, stop_idx, v_init, timestep, duration, seed):
        """
        Simulate a crash any time step i in between the time step values in window = (start, end) because of a faulty s
        communication. This selection will be random.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param window: The start and end time step of the V2I comm and the index where v = 0 (tuple).
        :param stop_idx: The index i within self.v2i_comms that contains a specific s communication.
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory. (int)
        :return: A pandas dataframe equal to the truth trajectory up until i when the crash occurs.
        """
        self.v2i_comms.pop(stop_idx)
        faulty = self.traj(v_init, timestep, duration, seed)
        start, end, i_when_v_is_0 = window
        i = random.randint(i_when_v_is_0, end)

        faulty.iloc[i:, 2] = 0
        faulty.iloc[i:, 3] = 0
        x_prev = truth.at[i - 1, "position"]
        faulty.iloc[i:, 1] = x_prev
        return faulty

    def simulate_crash_rs(self, truth, window, rs_idx, v_init, timestep, duration, seed):
        """
        Simulate a crash any time step i in between the time step values in window = (start, end) because of a faulty rs
        communication. This selection will be random.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param window: The start and end time step of the V2I comm and the index where v = 0 (tuple).
        :param rs_idx: The index i within self.v2i_comms that contains a specific s communication.
        :param v_init: The initial velocity of the vehicle, in m/s (int).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory, in seconds (int).
        :param seed: The seed of the benign trajectory. (int)
        :return: A pandas dataframe equal to the truth trajectory up until i when the crash occurs.
        """
        self.v2i_comms.pop(rs_idx)
        faulty = self.traj(v_init, timestep, duration, seed)
        start, end, i_when_v_is_des_v = window
        i = random.randint(i_when_v_is_des_v, end)

        faulty.iloc[i:, 2] = 0
        faulty.iloc[i:, 3] = 0
        x_prev = truth.at[i - 1, "position"]
        faulty.iloc[i:, 1] = x_prev
        return faulty

    def get_random_RS_comm(self):
        """
        Retrieve a random rs communication alongside its respective time step when the V2I communication
        was passed into the vehicle during its trajectory.
        
        :return: Tuple containing all 3 values about the random rs communication.
        """
        # Iteratively find an RS comm randomly within the v2i_comms
        rs_comm = "_,_,_,_"
        rs_idx_in_v2i = -1
        window = None
        while rs_comm.split(",")[0] != 'RS':
            rs_idx_in_v2i = random.choice(np.arange(len(self.v2i_comms)))
            rs_comm = self.v2i_comms[rs_idx_in_v2i]

        # Find the respective start and end time of this comm within self.vehicle
        for comm in self.vehicle.comms:
            if comm[0] == rs_comm:
                window = comm[1]
                break

        return rs_comm, rs_idx_in_v2i, window

    def get_random_S_comm(self):
        """
        Retrieve a random s communication alongside its respective time step when the V2I communication
        was passed into the vehicle during its trajectory.
        
        :return: Tuple containing all 3 values about the random s communication.
        """
        # Iteratively find an S comm randomly within the v2i_comms
        # WE ASSUME THERE EXIST ONE S comm within v2i_comms
        stop_comm = "_,_,_"
        stop_idx_in_v2i = -1
        window = None
        while stop_comm.split(",")[0] != 'S':
            stop_idx_in_v2i = random.choice(np.arange(len(self.v2i_comms)))
            stop_comm = self.v2i_comms[stop_idx_in_v2i]

        # Find the respective index of this comm within self.vehicle
        for comm in self.vehicle.comms:
            if comm[0] == stop_comm:
                window = comm[1]
                break

        return stop_comm, stop_idx_in_v2i, window

    def get_random_RS_info(self):
        """
        Retrieve all necessary information about the rs communication:
        1. The V2I rs communication.
        2. The index i of a V2I rs communication in self.v2i_comms
        3. A tuple of values = (start, end) where start = time step i when the V2I comm was passed into the vehicle
           during its trajectory and end = time step i when the V2I comm was finished executing.
        4. The distance to the work zone.
        5. The reduced speed in the work zone.
        6. The length/distance of the work zone.
        
        :return: Tuple containing all 6 values about the rs communication.
        """
        # Get a random RS com randomly within v2i comms
        rs_comm, rs_idx_in_v2i, window = self.get_random_RS_comm()
        # Get information from rs_comm
        dist_to_WZ, reduced_speed, len_of_WZ = map(int, rs_comm.split(",")[1:])
        return rs_comm, rs_idx_in_v2i, window, dist_to_WZ, reduced_speed, len_of_WZ

    def get_random_S_info(self):
        """
        Retrieve all necessary information about the s communication:
        1. The V2I s communication.
        2. The index i of a V2I s communication in self.v2i_comms
        3. A tuple of values = (start, end) where start = time step i when the V2I comm was passed into the vehicle
           during its trajectory and end = time step i when the V2I comm was finished executing.
        4. The distance to the work zone.
        5. The duration of the stop at the work zone.
        
        :return: Tuple containing all 5 values about the s communication.
        """
        # Get a random RS com randomly within v2i comms
        stop_comm, stop_idx_in_v2i, window = self.get_random_S_comm()
        # Get information from stop_comm
        dist_to_WZ, dur_of_WZ, = map(int, stop_comm.split(",")[1:])
        return stop_comm, stop_idx_in_v2i, window, dist_to_WZ, dur_of_WZ

    def perturb_rs_comm(self, dist_to_WZ, reduced_speed_of_WZ, len_of_WZ):
        """
        Return the perturbed rs communication given either:
        1. The perturbed dist_to_WZ, perturbed reduced_speed_of_WZ, and benign len_of_WZ.
        2. The perturbed dist_to_WZ, benign reduced_speed_of_WZ, and perturbed len_of_WZ.
        3. The benign dist_to_WZ, perturbed reduced_speed_of_WZ, and perturbed len_of_WZ.
        4. The perturbed dist_to_WZ, perturbed reduced_speed_of_WZ, and perturbed len_of_WZ.
        
        :param dist_to_WZ: The distance to the work zone, in meters (int).
        :param reduced_speed_of_WZ: The reduced speed in the work zone, in m/s (int).
        :param len_of_WZ: The length/distance of the work zone, in meters (int).
        :return: Perturbed V2I RS communication
        """
        perturbed_comm = 'RS,' + str(dist_to_WZ) + ',' + str(reduced_speed_of_WZ) + ',' + str(len_of_WZ)
        return perturbed_comm

    def perturb_s_comm(self, dist_to_WZ, dur_of_WZ):
        """
        Return a perturbed s communication given either:
        1. The perturbed dist_to_WZ and benign dur_of_WZ
        2. The benign dist_to_WZ and perturbed dur_of_WZ
        3. The perturbed dist_to_WZ and perturbed dur_of_WZ

        :param dist_to_WZ: The distance to the work zone, in meters (int).
        :param dur_of_WZ: The duration of the stop at the work zone, in seconds (int).
        :return: The perturbed V2I s communication
        """
        perturbed_comm = 'RS,' + str(dist_to_WZ) + ',' + str(dur_of_WZ)
        return perturbed_comm
