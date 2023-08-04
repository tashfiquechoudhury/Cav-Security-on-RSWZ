import pandas as pd
import numpy as np
import random
import matplotlib
from vehicle.vehicle import *


class Attack:

    def __init__(self, v_max, a_max):
        self.vehicle = Vehicle(v_max, a_max)
        self.v2i_comms = ["S,100,20", "RS,100,10,500"]
        self.attks = \
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
        self.vehicle.trajectory(v_init, timestep, duration, self.v2i_comms, seed=seed)
        return self.vehicle.report()

    def compare(self, v_init, timestep, duration, seed, scenario=1):
        assert scenario in self.attks, "Choose a valid scenario from 0-9"
        random.seed(seed)
        benign = self.traj(v_init, timestep, duration, seed=seed)
        faulty = self.attks[scenario](v_init, timestep, duration, seed)

    def ignore_stop(self, truth, v_init, timestep, duration, seed):
        v2i_comms = list(filter(lambda comm: comm[0] != 'S', self.v2i_comms))
        if len(v2i_comms) == 0:
            return self.eq(truth, v_init, timestep, duration, seed)
        ignored_comm = random.choice(v2i_comms)

        # TODO: Find a way to determine the start time of the randomly selected STOP v2i comm.

        # Ignoring a stop will always result in a crash. Therefore, up until some comm. of stop, copy the dataframe
        # of the benign case and anything from there on, a[i] = 0, v[i] = 0, and pos[i] = pos[i - 1]

    def ignore_rs(self, truth, v_init, timestep, duration, seed):
        v2i_comms = list(filter(lambda comm: comm[0] != 'RS', self.v2i_comms))
        if len(v2i_comms) == 0:
            return self.eq(truth, v_init, timestep, duration, seed)
        ignored_comm = random.choice(v2i_comms)
        # Ignoring a rs will always result in a crash. Therefore, up until the first comm. of rs, copy the dataframe
        # of the benign case and anything from there on, a[i] = 0, v[i] = 0, and pos[i] = pos[i - 1]

    def swz_rs(self, truth, v_init, timestep, duration, seed):
        return

    def dwz_rs(self, truth, v_init, timestep, duration, seed):
        return

    def lwz_rs(self, truth, v_init, timestep, duration, seed):
        return

    def rswz(self, truth, v_init, timestep, duration, seed):
        return

    def dwz_stop(self, truth, v_init, timestep, duration, seed):
        return

    def dur_wz_stop(self, truth, v_init, timestep, duration, seed):
        return

    def stop(self, truth, v_init, timestep, duration, seed):
        return

    def eq(self, truth, v_init, timestep, duration, seed):
        return
