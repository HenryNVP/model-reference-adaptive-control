import numpy as np
import matplotlib.pyplot as plt
from robot_parameters import r, W, M, I, x0, y0, theta0, v0, omega0
from mrac_parameters import dt, Am, Bm, Ap, Bp, beta, P
from collections import deque


class Robot:
    def __init__(self, controller="dynamic"):
        """
        Initialize the robot
        """
        self.x = float(x0)  # Initial x position, [m]
        self.y = float(y0)  # Initial y position, [m]
        self.theta = float(theta0)  # Initial orientation, [radians]
        self.v = float(v0)  # Initial linear velocity, [m/s]
        self.omega = float(omega0)  # Initial angular velocity, [rad/s]

        if controller not in ["kinematic", "dynamic"]:
            print("Invalid controller type. Defaulting to dynamic.")
            self.controller = "dynamic"  # Set to default if invalid
        else:
            self.controller = controller

        # Initialize history to store robot's trajectory (x, y, theta) for each iteration
        self.history = deque(maxlen=10000)
        self.history.append((self.x, self.y, self.theta))

        # Initialize model reference and adaptive law parameters
        # Initial state of the reference model (example)
        self.xm = np.array([[0], [0]])
        # Initial parameter estimates for adaptive law
        self.kx = np.array([[0, 0], [0, 0]])
        # Initial parameter estimates for adaptive law
        self.kv = np.array([[0, 0], [0, 0]])

    def update_position(self, v, omega):
        """
        Update the robot's position and orientation based on its velocity and angular velocity.
        """
        self.x += v * np.cos(self.theta + omega * dt / 2) * dt
        self.y += v * np.sin(self.theta + omega * dt / 2) * dt
        self.theta += omega * dt

        # Record the new position for plotting
        self.history.append((float(self.x), float(self.y), float(self.theta)))

    def kinematic_controller(self, EX, EY):
        """
        Implements a kinematic controller based on tracking error
        The controller is a quadratic curve controller
        """

        alpha = 1     # Quadratic curve controller speed constant

        ex = np.cos(self.theta) * EX + np.sin(self.theta) * EY
        ey = -np.sin(self.theta) * EX + np.cos(self.theta) * EY

        Aq = np.sign(ex) * ey / (ex ** 2) if ex != 0 else 0
        K = np.sign(ex) * alpha / (1 + abs(Aq)) if ex != 0 else alpha

        v = K  # Linear velocity
        omega = 2 * Aq * K  # Angular velocity

        return v, omega

    def dynamic_controller(self, EX, EY):
        """
        Implements a dynamic controller based on tracking error
        The controller is model reference adaptive controller (MRAC)
        """

        # Get kinematic control inputs
        vk, omegak = self.kinematic_controller(EX, EY)
        vd = np.array([[vk], [omegak]])  # Desired velocity (2x1 matrix)
        vd = vd.reshape(2, 1)

        # Model reference dynamics (MRAC)
        xm = np.dot(Am, self.xm) + np.dot(Bm, vd)
        self.xm = xm  # Update xm for next iteration

        # Adaptive law
        # Current state vector of the robot
        xp = np.array([[self.v], [self.omega]])
        em = xp - xm  # Tracking error

        # Update parameter estimates (adaptive law)
        self.kx = self.kx - beta * P @ (em * np.transpose(xp)) * dt
        self.kv = self.kv - beta * P @ (em * np.transpose(vd)) * dt

        # Control input (model reference adaptive control)
        # Voltage inputs to motors
        u = np.dot(self.kx, xp) + np.dot(self.kv, vd)

        # Update the state based on the control input
        xp = np.dot(Ap, xp) + np.dot(Bp, u)  # Update the state of the system
        # Extract the velocity and angular velocity from the state
        v, omega = xp[0], xp[1]

        return v, omega

    def move_to_reference(self, xref, yref):
        """
        Move the robot towards a reference point (xref, yref).
        """

        travel_time = 0

        EX = xref - self.x
        EY = yref - self.y
        iteration = 0  # To prevent infinite loops for debugging

        # Select the controller function once based on the controller type
        if self.controller == "kinematic":
            controller_func = self.kinematic_controller
        else:
            controller_func = self.dynamic_controller

        while np.sqrt(EX ** 2 + EY ** 2) > 0.05:
            # Debugging prints to check the values of EX, EY, position, and iteration count
            # print(f"Iteration {iteration}: EX={EX}, EY={EY}, x={self.x}, y={self.y}, theta={self.theta}")

            # Update velocity and angular velocity
            v, omega = controller_func(EX, EY)
            self.update_position(v, omega)  # Update robot position
            travel_time += dt

            EX = xref - self.x  # Recalculate EX and EY after moving
            EY = yref - self.y
            iteration += 1
            if iteration > 10000:  # Fail-safe to avoid infinite loops
                print("Max iterations reached. Exiting.")
                break

        print("Reached destination")

        return travel_time

    def plot_path(self):
        """
        Plot the robot's trajectory
        """

        # Check if we have any valid data to plot
        if len(self.history) == 0:
            print("Error: No valid history to plot.")
            return

        # Convert the filtered history to a numpy array
        try:
            # Flatten the history list into a numpy array of shape (n, 3)
            history_array = np.array(self.history, dtype=float)
        except Exception as e:
            print(f"Error while converting history to array: {e}")
            return

        # Ensure the resulting history_array has the expected shape (n, 3)
        if history_array.shape[1] != 3:
            print(
                f"Error: history_array has an unexpected shape: {history_array.shape}")
            return

        plt.figure()
        # Plot the robot's path (x, y coordinates)
        plt.plot(history_array[:, 0], history_array[:, 1],
                 marker='.', markersize=2)
        plt.title(f"Robot Path. Controller type: {self.controller}")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.grid(True)
        plt.axis("equal")
        plt.show()
