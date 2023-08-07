import random
from vehicle.vehicle import *


# TODO: Add documentation
class Attack:

    def __init__(self, v_max, a_max):
        """

        @param v_max:
        @param a_max:
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

        @param v_init:
        @param timestep:
        @param duration:
        @param seed:
        @return:
        """
        self.vehicle.trajectory(v_init, timestep, duration, self.v2i_comms, seed=seed)
        return self.vehicle.report()

    def compare(self, v_init, timestep, duration, seed, scenario=1):
        """

        @param v_init:
        @param timestep:
        @param duration:
        @param seed:
        @param scenario:
        @return:
        """
        assert scenario in self.attack_panel, "Choose a valid attack mechanism ranging from 0-9"

        random.seed(seed)
        benign_traj = self.traj(v_init, timestep, duration, seed=seed)
        faulty_traj = None

        if scenario == 0:
            faulty_traj = self.eq(benign_traj)
        elif scenario == 1:
            faulty_traj = self.ignore_stop(benign_traj)
        elif scenario == 2:
            faulty_traj = self.ignore_rs(benign_traj)
        elif scenario == 3:
            perturbed_v = input("Input perturbed reduced speed in work zone: ")
            faulty_traj = self.swz_rs(benign_traj, v_init, perturbed_v, timestep, duration, seed)
        elif scenario == 4:
            perturbed_dist = input("Input perturbed distance to work zone: ")
            faulty_traj = self.dwz_rs(benign_traj, v_init, perturbed_dist, timestep, duration, seed)
        elif scenario == 5:
            perturbed_len = input("Input perturbed length of work zone: ")
            faulty_traj = self.lwz_rs(benign_traj, v_init, perturbed_len, timestep, duration, seed)
        elif scenario == 6:
            perturbed = input(
                "Input perturbed reduced speed, distance, and length of work zone as csv in that order: ").split(",")
            faulty_traj = self.rswz(benign_traj, v_init, perturbed, timestep, duration, seed)
        elif scenario == 7:
            perturbed_dist = input("Input perturbed distance to work zone: ")
            faulty_traj = self.dwz_stop(benign_traj, perturbed_dist)
        elif scenario == 8:
            perturbed_dur = input("Input perturbed duration of work zone: ")
            faulty_traj = self.dur_wz_stop(benign_traj, v_init, perturbed_dur, timestep, duration, seed)
        elif scenario == 9:
            perturbed = input("Input perturbed distance and duration of work zone as csv in that order: ").split(",")
            faulty_traj = self.stop(benign_traj, v_init, perturbed, timestep, duration, seed)

        return faulty_traj, benign_traj

        # With both trajectories generated, we will now compare them with graphs!
        # TODO: Visualize the difference(s) between the two trajectories of faulty and benign and save them.

    # Attack panel

    def ignore_stop(self, truth):
        """
        Ignore one V2I stop communication.

        @param truth:
        @return: faulty trajectory
        """

        # Get a random S comm randomly within v2i_comms
        _, stop_idx, _, _ = self.get_random_S_info()

        return self.simulate_crash(truth, stop_idx)

    def ignore_rs(self, truth):
        """
        Ignore one V2I rs communication.

        @param truth:
        @return: faulty trajectory
        """

        # Get a random RS comm randomly within v2i_comms
        _, rs_idx, _, _, _ = self.get_random_RS_info()

        return self.simulate_crash(truth, rs_idx)

    def swz_rs(self, truth, v_init, perturbed_v, timestep, duration, seed):
        """
        Change the speed of the reduced speed in the work zone. We have two scenarios:
        (1) Nothing happens, and we just see a change in the speed during a work zone
        (2) We crash UNLESS the perturbed_v <= speed of the rs_comm
        @param truth:
        @param v_init:
        @param perturbed_v:
        @param timestep:
        @param duration:
        @param seed:
        @return:
        """

        outcome = random.choice([0, 1])
        rs_comm, rs_idx, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        if outcome == 1:
            # (1): Find one RS comm and perturb it. Everything other comm stays the same.
            # Create our perturbed v2i_comms
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(dist_to_WZ, perturbed_v, len_of_WZ)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty
        elif outcome == 2:
            # (2): Crash
            # If our perturbed v is less than the actual reduce speed in the work zone, no crash should happen.
            if rs_comm.split(",")[2] >= perturbed_v:
                return self.eq(truth)
            else:
                return self.simulate_crash(truth, rs_idx)

    def dwz_rs(self, truth, v_init, perturbed_dist, timestep, duration, seed):
        """
        Change the distance to the work zone. We have two scenarios:
        (1) Crash:
            (1a): Crash because the actual distance is smaller than perturbed distance since we're not reducing our
                  speed prior to entering the work zone.
            (1b): Crash because the actual distance to the work zone near len_of_WZ (unchanged) + perturbed_dist which
                  means our vehicle will traverse the entire "work zone" before even entering the work zone.
        (2) Nothing happens:
            (2a): Nothing happens because the actual distance is larger than the perturbed distance since we're reducing
                  our speed then speeding up and the discrepancy of speeding up can be non-trivial thus we randomize the
                  outcome.
        @param truth:
        @param v_init:
        @param perturbed_dist:
        @param timestep:
        @param duration:
        @param seed:
        @return:
        """

        outcome = random.choice([0, 1])
        rs_comm, rs_idx, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        # Case 1a & 2b & 2
        if dist_to_WZ <= perturbed_dist or dist_to_WZ - perturbed_dist - len_of_WZ <= 10 or outcome == 0:
            return self.simulate_crash(truth, rs_idx)
        else:
            # No crash but our trajectory is perturbed
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(perturbed_dist, reduced_speed, len_of_WZ)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def lwz_rs(self, truth, v_init, perturbed_len, timestep, duration, seed):
        """
        Change the length of the work zone.
        1. Crash because the perturbed_length <= actual length
        2. No crash because the perturbed_length >= actual length but traj changes

        @param truth:
        @param v_init:
        @param perturbed_len:
        @param timestep:
        @param duration:
        @param seed:
        @return:
        """

        rs_comm, rs_idx, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        if len_of_WZ > perturbed_len:
            return self.simulate_crash(truth, rs_idx)
        else:
            # No crash but our trajectory is perturbed
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(dist_to_WZ, reduced_speed, perturbed_len)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def rswz(self, truth, v_init, perturbed, timestep, duration, seed):
        """
        Change the length of, distance to, and speed limit of work zone.

        Ultimately, we could either crash have a perturbed communication that would give us two different trajectories.
        It'll be crash.

        @param truth:
        @param v_init:
        @param perturbed:
        @param timestep:
        @param duration:
        @param seed:
        @return:
        """
        outcome = random.choice([0, 1])
        rs_comm, rs_idx, dist_to_WZ, reduced_speed, len_of_WZ = self.get_random_RS_info()

        if outcome == 0:
            return self.simulate_crash(truth, rs_idx)
        else:
            # Unpack perturbed values
            perturbed_dist, perturbed_v, perturbed_len = perturbed
            self.v2i_comms[rs_idx] = self.perturb_rs_comm(perturbed_dist, perturbed_v, perturbed_len)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def dwz_stop(self, truth, perturbed_dist):
        """
        Change the distance to the work zone.

        When it comes to making a stop, we either stop exactly where we need to or we crash.
        Why crash?
        1. We stop before under the assumption that's the stop zone and crash as we move into the actual stop zone
        2. We stop after crossing the stop zone but we crash regardless.

        @param truth:
        @param perturbed_dist:
        @return:
        """
        stop_comm, stop_idx, dist_to_WZ, dur_of_WZ = self.get_random_S_info()

        # Case 1 and 2
        if dist_to_WZ - perturbed_dist > 5:
            return self.eq(truth)
        else:
            return self.simulate_crash(truth, stop_idx)

    def dur_wz_stop(self, truth, v_init, perturbed_dur, timestep, duration, seed):
        """
        Change the duration of the stop.
        1. If we go over the actual duration, it doesn't really matter perhaps? (Maybe crash)
        2. Waiting less than the actual duration is crash.


        @param truth:
        @param v_init:
        @param perturbed_dur:
        @param timestep:
        @param duration:
        @param seed:
        @return:
        """
        outcome = random.choice([0, 1])
        stop_comm, stop_idx, dist_to_WZ, dur_of_WZ = self.get_random_S_info()
        if 0 < dur_of_WZ - perturbed_dur <= 1 or perturbed_dur > dur_of_WZ and outcome == 0:
            return self.simulate_crash(truth, stop_idx)
        else:
            self.v2i_comms[stop_idx] = self.perturb_s_comm(dist_to_WZ, perturbed_dur)
            faulty = self.traj(v_init, timestep, duration, seed)
            return faulty

    def stop(self, truth, v_init, perturbed, timestep, duration, seed):
        """
        Change the distance to and duration of the stop at the work zone.

        Ultimately, we either crash or have two different trajectories.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param v_init: The initial velocity of the vehicle (int).
        :param perturbed: The perturbed communications (list).
        :param timestep: The timestep of the benign trajectory (int).
        :param duration: The duration of the benign trajectory (int).
        :param seed: The seed of the beinign treajectyory. (int)
        :return:
        """

        outcome = random.choice([0, 1])
        stop_comm, stop_idx, dist_to_WZ, dur_of_WZ = self.get_random_S_info()

        if outcome == 0:
            return self.simulate_crash(truth, stop_idx)
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

    def simulate_crash(self, truth, i):
        """
        Simulate a crash starting at time step i.

        :param truth: The benign trajectory of the CAV (Pandas DataFrame).
        :param i: The time step i when the crash occurs (int).
        :return: A pandas dataframe equal to the truth trajectory up until i when the crash occurs.
        """

        faulty = truth.copy()
        faulty.iloc[i:, 2] = 0
        faulty.iloc[i:, 3] = 0
        x_prev = truth.at[i - 1, "position"]
        faulty.iloc[i:, 1] = x_prev

        return faulty

    def get_random_RS_comm(self):
        """
        Retrieve a random rs communication alongside its respective time step when the V2I communication
        was passed into the vehicle during its trajectory.
        
        :return: Tuple containing all 2 values about the random rs communication.
        """
        # Iteratively find an RS comm randomly within the v2i_comms
        rs_comm = "_,_,_,_"
        rs_idx = -1
        while rs_comm.split(",")[0] != 'RS':
            rs_idx = random.choice(np.arange(len(self.v2i_comms)))
            rs_comm = self.v2i_comms[rs_idx]

        # Find the respective index of this comm within self.vehicle
        for comm in self.vehicle.comms:
            if comm[0] == rs_comm:
                rs_idx = comm[1]
                break

        return rs_comm, rs_idx

    def get_random_S_comm(self):
        """
        Retrieve a random s communication alongside its respective time step when the V2I communication
        was passed into the vehicle during its trajectory.
        
        :return: Tuple containing all 2 values about the random s communication.
        """
        # Iteratively find an S comm randomly within the v2i_comms
        # WE ASSUME THERE EXIST ONE S comm within v2i_comms
        stop_comm = "_,_,_"
        stop_idx = -1
        while stop_comm.split(",")[0] != 'S':
            stop_idx = random.choice(np.arange(len(self.v2i_comms)))
            stop_comm = self.v2i_comms[stop_idx]

        # Find the respective index of this comm within self.vehicle
        for comm in self.vehicle.comms:
            if comm[0] == stop_comm:
                stop_idx = comm[1]
                break

        return stop_comm, stop_idx

    def get_random_RS_info(self):
        """
        Retrieve all necessary information about the rs communication:
        1. The V2I rs communication.
        2. The time step i when the V2I rs communication was passed into the vehicle during its trajectory.
        3. The distance to the work zone.
        4. The reduced speed in the work zone.
        5. The length/distance of the work zone.
        
        :return: Tuple containing all 5 values about the rs communication.
        """
        # Get a random RS com randomly within v2i comms
        rs_comm, rs_idx = self.get_random_RS_comm()
        # Get information from rs_comm
        dist_to_WZ, reduced_speed, len_of_WZ = map(int, rs_comm.split(",")[1:])
        return rs_comm, rs_idx, dist_to_WZ, reduced_speed, len_of_WZ

    def get_random_S_info(self):
        """
        Retrieve all necessary information about the s communication:
        1. The V2I s communication.
        2. The time step i when the V2I s communication was passed into the vehicle during its trajectory.
        3. The distance to the work zone.
        4. The duration of the stop at the work zone.
        
        :return: Tuple containing all 4 values about the s communication.
        """
        # Get a random RS com randomly within v2i comms
        stop_comm, stop_idx = self.get_random_S_comm()
        # Get information from stop_comm
        dist_to_WZ, dur_of_WZ, = map(int, stop_comm.split(",")[1:])
        return stop_comm, stop_idx, dist_to_WZ, dur_of_WZ

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
        3. The peturbed dist_to_WZ and perturbed dur_of_WZ

        :param dist_to_WZ: The distance to the work zone, in meters (int).
        :param dur_of_WZ: The duration of the stop at the work zone, in seconds (int).
        :return: The perturbed V2I s communication
        """
        perturbed_comm = 'RS,' + str(dist_to_WZ) + ',' + str(dur_of_WZ)
        return perturbed_comm
