import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import format


# TODO: Create another trajectory function that will map 2D instead of 1D (that
#  means, we need velocity_x, velocity_y, acceleration_x, acceleration_y,
#  position_x, and position_y)

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
        pos_x[i] = pos_x[i - 1] + t[i - 1] * v_x[i - 1] + (0.5 * a_x[i - 1] * (t[i - 1] ** 2))
        v_x[i] = v_x[i - 1] + t[i - 1] * a_x[i - 1]
        t[i] = t[i - 1] + tau

        # TODO: Create a more complex algorithm for acceleration.
        # Assume CAV is accelerating for a quarter of the trip, then decelerating for a quarter of the trip.
        # Then accelerating for the rest of the trip.
        if i < N / 4:
            a_x[i] = a_x[i - 1] + 0.0001
        elif i < N / 2:
            a_x[i] = a_x[i - 1] - 0.00005
        else:
            a_x[i] = a_x[i - 1] + 0.0001

    # Aggregate and combine data into one dataframe.
    data = np.column_stack((t, pos_x, v_x, a_x))
    dataframe = pd.DataFrame(data, columns=['time', 'position', 'velocity', 'acceleration'])
    return dataframe


def generate_trajectory_2D(total_duration, timestep, init_velocity_x, init_velocity_y):
    """
    Returns the velocity, position, and acceleration of the CAV at each timestep up until total_duration as a Pandas
    Dataframe in 2D.

    Format:
            time    x-position  y-position  x-velocity  y-velocity  x-acceleration  y-acceleration
             0
             tau
             2tau
             3tau
             ...
             N

    where N = total_duration

    :param total_duration:
    :param timestep:
    :param init_velocity_x:
    :param init_velocity_y:
    """
    # Initialize velocity, acceleration, position, and time arrays.
    tau = timestep
    N = int(total_duration / tau) + 1
    t = np.zeros(N, dtype=np.float32)
    pos_x = np.zeros(N, dtype=np.float32)
    pos_y = np.zeros(N, dtype=np.float32)
    v_x = np.zeros(N, dtype=np.float32)
    v_y = np.zeros(N, dtype=np.float32)
    a_x = np.zeros(N, dtype=np.float32)
    a_y = np.zeros(N, dtype=np.float32)

    # Initialize initial velocity
    v_x[0] = init_velocity_x
    v_y[0] = init_velocity_y

    # Simulate trajectory of CAV
    for i in range(1, N):
        pos_x[i] = pos_x[i - 1] + t[i - 1] * v_x[i - 1] + (0.5 * a_x[i - 1] * (t[i - 1] ** 2))
        pos_y[i] = pos_y[i - 1] + t[i - 1] * v_y[i - 1] + (0.5 * a_y[i - 1] * (t[i - 1] ** 2))
        v_x[i] = v_x[i - 1] + t[i - 1] * a_x[i - 1]
        v_y[i] = v_y[i - 1] + t[i - 1] * a_y[i - 1]
        t[i] = t[i - 1] + tau

        # TODO: Create a more complex algorithm for acceleration.
        # Assume CAV is accelerating for a quarter of the trip, then decelerating for a quarter of the trip.
        # Then accelerating for the rest of the trip.
        if i < N / 4:
            a_x[i] = a_x[i - 1] + 0.0001
        elif i < N / 2:
            a_x[i] = a_x[i - 1] - 0.00005
        else:
            a_x[i] = a_x[i - 1] + 0.0001

    # Aggregate and combine data into one dataframe.
    data = np.column_stack((t, pos_x, pos_y, v_x, v_y, a_x, a_y))
    dataframe = pd.DataFrame(data, columns=['time', 'x-position', 'y-position', 'x-velocity',
                                            'y-velocity', 'x-acceleration', 'y-acceleration'])
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
