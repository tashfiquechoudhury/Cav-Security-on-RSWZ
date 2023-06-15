from format import *
from vehicle import *

nissan = Vehicle(40, 6)
plot_trajectory_1D(nissan.generate_trajectory_1D(100, 5, 0))
