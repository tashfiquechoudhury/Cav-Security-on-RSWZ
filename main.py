from approach_1c import vehicle
from format import *

corolla = vehicle.Vehicle(70, 4, "corolla", "toyota")
corolla.trajectory(1, 1, 100, 1)
traj = corolla.report()
print(traj)
plot_trajectory_1D(traj)
