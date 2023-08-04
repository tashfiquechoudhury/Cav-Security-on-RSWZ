import matplotlib.pyplot as plt
import matplotlib.ticker as mticker


class MathTextSciFormatter(mticker.Formatter):
    """
    A class that will help format the values of the axes for the graphs using LaTeX.
    """

    def __init__(self, fmt="%1.2e"):
        self.fmt = fmt

    def __call__(self, x, pos=None):
        s = self.fmt % x
        decimal_point = '.'
        positive_sign = '+'
        tup = s.split('e')
        significand = tup[0].rstrip(decimal_point)
        sign = tup[1][0].replace(positive_sign, '')
        exponent = tup[1][1:].lstrip('0')
        if exponent:
            exponent = '10^{%s%s}' % (sign, exponent)
        if significand and exponent:
            s = r'%s{\times}%s' % (significand, exponent)
        else:
            s = r'%s%s' % (significand, exponent)
        return "${}$".format(s)

def plot_trajectory_1D(dataframe):
    """
    Displays and visualizes the trajectory of the CAV given a Pandas DataFrame.

    :param dataframe:
    """
    # Acceleration
    dataframe.plot(kind='line', x='time', y='acceleration', color='green')
    plt.gca().yaxis.set_major_formatter(MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    plt.xlabel("Time (s)", size=15)
    plt.ylabel("Acceleration (m/s^2)", size=15)
    plt.title("Acceleration of CAV", size=20)

    plt.show()

    # Position
    dataframe.plot(kind='line', x='time', y='position', color='green')
    plt.gca().yaxis.set_major_formatter(MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    plt.xlabel("Time (s)", size=15)
    plt.ylabel("Position (x-coordinate)", size=15)
    plt.title("Position of CAV", size=20)

    plt.show()

    # Velocity
    dataframe.plot(kind='line', x='time', y='velocity', color='green')
    plt.gca().yaxis.set_major_formatter(MathTextSciFormatter("%1.2e"))
    plt.gca().xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    plt.xlabel("Time (s)", size=15)
    plt.ylabel("Velocity (m/s)", size=15)
    plt.title("Velocity of CAV", size=20)

    plt.show()
