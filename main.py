from vehicle.attack import *
from format import *

attk = Attack(70, 4)
faulty, benign = attk.compare(1, 1, 500, 10, scenario=2)

plot_trajectory_1D(faulty)
plot_trajectory_1D(benign)
