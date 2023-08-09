from vehicle.attack import *
from format import *

attk = Attack(70, 4)
faulty, benign = attk.compare(1, 1, 500, 20, scenario=1)

# plot_trajectory_1D(faulty)
# plot_trajectory_1D(benign)  
plot_trajectory_compare(faulty, benign)