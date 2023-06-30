from vehicle import *

"""
A file that will consist of unit tests for the functionality of the trajectory method of a vehicle. For e.g.,
make sure the acceleration phase, random phase, and deceleration phase all work properly.
"""
acceleration = Vehicle(70, 4, "corolla", "toyota").trajectory(1, 5, 100, 2)
print(acceleration)
