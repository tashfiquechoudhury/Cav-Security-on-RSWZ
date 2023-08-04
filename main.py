from vehicle.attack import *
from format import *

attk = Attack(70, 4)
traj = attk.benign(1, 1, 500, 10)
plot_trajectory_1D(traj)
