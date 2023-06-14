import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import format


# TODO: Make a vehicle class and acceleration class.

# This function only calculates the 1D position of the vehicle (x-coordinate)
def generate_trajectory_1D(total_duration, timestep, init_velocity):
    """
    Returns the velocity, position, and acceleration of the CAV at each timestep up until total_duration as a Pandas
    Dataframe in 1D.

    Format:
            time    position    velocity    acceleration
             0
             tau
             2tau
             3tau
             ...
             N

    where N = total_duration

    :param total_duration:
    :param timestep:
    :param init_velocity:
    """
    # Initialize velocity, acceleration, position, and time arrays.
    tau = timestep
    N = int(total_duration / tau) + 1
    t = np.zeros(N, dtype=np.float32)
    pos_x = np.zeros(N, dtype=np.float32)
    v_x = np.zeros(N, dtype=np.float32)
    a_x = np.zeros(N, dtype=np.float32)

    # Initialize initial velocity
    v_x[0] = init_velocity

    # Simulate trajectory of CAV
    for i in range(1, N):
        # wait user input  = input()
        # use this input to change our acceleration
        # do something
        pos_x[i] = pos_x[i - 1] + t[i - 1] * v_x[i - 1] + (0.5 * a_x[i - 1] * (t[i - 1] ** 2))
        v_x[i] = v_x[i - 1] + t[i - 1] * a_x[i - 1]
        t[i] = t[i - 1] + tau

        # TODO: Create a more complex algorithm for acceleration.
        # Assume CAV is accelerating for a quarter of the trip, then decelerating for a quarter of the trip.
        # Then accelerating for the rest of the trip.
        if i < N / 4:
            a_x[i] = a_x[i - 1] + 0.1
        elif i < N / 2:
            a_x[i] = a_x[i - 1] - 0.05
        else:
            a_x[i] = a_x[i - 1] + 0.1

    # Aggregate and combine data into one dataframe.
    data = np.column_stack((t, pos_x, v_x, a_x))
    dataframe = pd.DataFrame(data, columns=['time', 'position', 'velocity', 'acceleration'])
    return dataframe


# This function will display the trajectory of the CAV in 1D (x-coordinate)
# TODO: Fix plot axes
def plot_trajectory_1D(dataframe):
    """
    Displays and visualizes the trajectory of the CAV given a Pandas DataFrame.

    :param dataframe:
    """
    # Acceleration
    dataframe.plot(kind='scatter', x='time', y='acceleration', color='green')
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(format.MaxNLocator(integer=True))
    plt.xlabel("Time (s)", size=15)
    plt.ylabel("Acceleration (m/s^2)", size=15)
    plt.title("Acceleration of CAV", size=20)

    plt.show()

    # Position
    dataframe.plot(kind='scatter', x='time', y='position', color='green')
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(format.MaxNLocator(integer=True))
    plt.xlabel("Time (s)", size=15)
    plt.ylabel("Position (x-coordinate)", size=15)
    plt.title("Position of CAV", size=20)

    plt.show()

    # Velocity
    dataframe.plot(kind='scatter', x='time', y='velocity', color='green')
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(format.MaxNLocator(integer=True))
    plt.xlabel("Time (s)", size=15)
    plt.ylabel("Velocity (m/s)", size=15)
    plt.title("Velocity of CAV", size=20)

    plt.show()
