from format import *
from approach_1a.vehicle import *


nissan = Vehicle(40, 6, 1)
plot_trajectory_1D(nissan.generate_trajectory_1D(100, 5, 0))
