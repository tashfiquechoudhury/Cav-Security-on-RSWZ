import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import format


# TODO: Design an algorithm that will vary the acceleration based on
#  the current time t to represent a vehicle's acceleration and deceleration
#  as if it were driving. (speeding up and slowing down, accelerate, and decelerate, respectively)

# TODO: Create another trajectory function that will map 2D instead of 1D (that
#  means, we need velocity_x, velocity_y, acceleration_x, acceleration_y,
#  position_x, and position_y)

# This function only calculates the 1D position of the vehicle (x-coordinate)
def generate_trajectory_1D(total_duration, timestep, init_velocity):
    """
    Returns the velocity, position, and acceleration of the CAV at each timestep up until total_duration as a Pandas
    Dataframe.

    Format:
                position    velocity    acceleration    timestep
                                                            1
                                                            2
                                                            ...
                                                            N

    where N = total_duration

    :param total_duration:
    :param timestep:
    :param init_velocity:
    """
    velocities = [init_velocity]
    accelerations = [1]
    positions = [0]
    timesteps = []
    tau = timestep

    for t in range(1, total_duration):
        position = positions[t - 1] + t * velocities[t - 1] + (0.5 * accelerations[t - 1] * (t ** 2))
        velocity = velocities[t - 1] + t * accelerations[t - 1]
        acceleration = accelerations[t - 1]

        positions.append(position)
        velocities.append(velocity)
        timesteps.append(t)

        if t % 10 == 0:
            acceleration = accelerations[t - 1] + 0.5
            accelerations.append(acceleration)
        else:
            accelerations.append(acceleration)

    data = np.array(list(zip(positions, velocities, accelerations, timesteps)), dtype=np.int32)
    dataframe = pd.DataFrame(data, columns=['position', 'velocity', 'acceleration', 'timestep'])
    return dataframe


# This function will display the trajectory of the CAV in 1D (x-coordinate)
def plot_trajectory_1D(dataframe):
    """
    Displays and visualizes the trajectory of the CAV given a Pandas DataFrame.

    :param dataframe:
    """
    # Acceleration
    dataframe.plot(kind='scatter', x='timestep', y='acceleration', color='green')
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(format.MaxNLocator(integer=True))
    plt.xlabel("Time step", size=15)
    plt.ylabel("Acceleration (m/s^2)", size=15)
    plt.title("Acceleration of CAV", size=20)

    plt.show()

    # Position
    dataframe.plot(kind='scatter', x='timestep', y='position', color='green')
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(format.MaxNLocator(integer=True))
    plt.xlabel("Time step", size=15)
    plt.ylabel("Position (x-coordinate)", size=15)
    plt.title("Position of CAV", size=20)

    plt.show()

    # Velocity
    dataframe.plot(kind='scatter', x='timestep', y='velocity', color='green')
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(format.MaxNLocator(integer=True))
    plt.xlabel("Time step", size=15)
    plt.ylabel("Velocity (m/s)", size=15)
    plt.title("Velocity of CAV", size=20)

    plt.show()


plot_trajectory_1D(generate_trajectory_1D(20, 1, 0.5))
