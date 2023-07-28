from approach_1c import vehicle
from format import *

corolla2 = vehicle.Vehicle(70, 4, "corolla", "toyota")
corolla2.trajectory(1, 1, 500, 10)
traj2 = corolla2.report()
print(traj2)
plot_trajectory_1D(traj2)
