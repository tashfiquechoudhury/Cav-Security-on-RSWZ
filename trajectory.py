import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import format


# This function only calculates the 1D position of the vehicle (x-coordinate)
def generate_trajectory_1D(total_duration, timestep, init_velocity):
    """
    Returns the velocity, position, and acceleration of the CAV at each timestep up until total_duration as a Pandas
    Dataframe.

    Format:
                position    velocity    acceleration
    timestep 1
    timestep 2
    ...
    timestep N

    where N = total_duration

    :param total_duration:
    :param timestep:
    :param init_velocity:
    """
    velocities = [init_velocity]
    accelerations = [1]
    positions = [0]
    tau = timestep

    for t in range(1, total_duration):
        position = positions[t - 1] + tau * velocities[t - 1] + (((tau ** 2) / 2) * accelerations[t - 1])
        velocity = velocities[t - 1] + tau * accelerations[t - 1]
        acceleration = accelerations[t - 1]

        positions.append(position)
        velocities.append(velocity)

        if t % 10 == 0:
            acceleration = accelerations[t - 1] + 0.5
            accelerations.append(acceleration)
        else:
            accelerations.append(acceleration)

    data = np.array(list(zip(positions, velocities, accelerations)), dtype=np.int32)
    dataframe = pd.DataFrame(data, columns=['position', 'velocity', 'acceleration'],
                             index=[i + 1 for i in range(0, total_duration)])
    return dataframe


# This function will display the trajectory of the CAV in 1D (x-coordinate)
def plot_trajectory_1D(dataframe):
    """
    Displays and visualizes the trajectory of the CAV given a Pandas DataFrame.

    :param dataframe:
    """
    # Acceleration
    dataframe.plot(y='acceleration', color='darkblue', use_index=True)
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.xlabel("Time step", size=15)
    plt.ylabel("Acceleration (m/s^2)", size=15)
    plt.title("Acceleration of CAV", size=20)

    # Position
    dataframe.plot(y='position', color='red', use_index=True)
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.xlabel("Time step", size=15)
    plt.ylabel("Position (x-coordinate)", size=15)
    plt.title("Position of CAV", size=20)

    # Velocity
    dataframe.plot(y='velocity', color='green', use_index=True)
    plt.gca().yaxis.set_major_formatter(format.MathTextSciFormatter("%1.2e"))
    plt.xlabel("Time step", size=15)
    plt.ylabel("Velocity (m/s)", size=15)
    plt.title("Velocity of CAV", size=20)

    plt.show()
