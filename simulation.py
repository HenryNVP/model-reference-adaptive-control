import numpy as np
from Robot import Robot


def simulation():
    # Create two robot instances with different controllers
    robot1 = Robot("dynamic")
    robot2 = Robot("kinematic")

    travel_time1 = 0
    travel_time2 = 0

    # Reference points
    Xref = np.array([0, 0.45, 0.9, 1.35, 1.8, 2.25, 2.7, 3.15,
                    2.7, 2.25, 1.8, 2.25, 2.7, 3.15, 3.6, 4.05, 4.5])
    Yref = np.array([-1.8, -1.8, -1.8, -1.8, -1.8, -1.8, -
                    1.8, -0.9, 0, 0, 0.9, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8])

    # Move the robots to each reference point
    for x, y in zip(Xref, Yref):
        segment_time1 = robot1.move_to_reference(x, y)
        travel_time1 += segment_time1

    for x, y in zip(Xref, Yref):
        segment_time2 = robot2.move_to_reference(x, y)
        travel_time2 += segment_time2

    # Plot the robots' paths after completing the movement
    robot1.plot_path()
    robot2.plot_path()

    print(
        f"Travel time of robot with model reference adaptive controller: {travel_time1:.1f}s")
    print(
        f"Travel time of robot with quadratic curve controller: {travel_time2:.1f}s")
