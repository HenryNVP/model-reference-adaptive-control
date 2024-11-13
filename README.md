# Mobile Robot Model Reference Adaptive Controller Simulation

## Overview
This project simulates the trajectory of a two-wheeled mobile robot using quadratic curve and model reference adaptive controller.

The project aims to replicate and visualize the proposed controllers to understand their effectiveness in path tracking.

## Features
- Simulates a two-wheeled robot's trajectory following a pre-defined path using only kinematic quadratic curve controller vs combining quadratic curve and model reference adaptive controller.
- Visualizes the robot's trajectory.

## Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/HenryNVP/model-reference-adaptive-control.git
   ```
2. Set up a Python virtual environment (optional but recommended):
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
   ```
3. Install the required packages:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
Run the simulation script to visualize the robot's path:
```bash
python main.py
```

## References
- *Indoor Virtual Path Tracking for Mobile Robot using Sensor Fusion by Extended Kalman Filter*
- *Development of a Quadratic Curve Path Tracking based Smith Predictor Adaptive Controller for a Two-wheeled Mobile Robot*

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contributions
Contributions are welcome! Feel free to open an issue or submit a pull request for improvements.

---

For further information or questions, please contact [henrynguyen.vp@gmail.com].

