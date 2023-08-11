from vehicle.attack import *
from format import *

attk = Attack(70, 4)
faulty, benign = attk.compare(1, 1, 500, 3, scenario=1)

# Scenario 6: 50, 5, 500
# faulty, benign = attk.compare(1, 1, 500, 5, scenario=6)

# Scenario 1: Ignore stop
# faulty, benign = attk.compare(1, 1, 500, 3, scenario=1)

plot_trajectory_compare(faulty, benign)