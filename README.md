# Project Title: UR5e Control GUI

This project provides a graphical user interface (GUI) for controlling a UR5e robot. The application allows users to connect to the robot, execute movements, log data, and export the logged data to CSV files.

## Files Overview

- **gui.py**: Contains the main GUI application for controlling the UR5e robot. It includes classes and methods for connecting to the robot, moving it, logging data, and exporting that data to CSV.

- **requirements.txt**: Lists the required Python packages for the project.

## Installation

To set up the project, ensure you have Python installed on your machine. Then, install the required packages using pip:

```
pip install -r requirements.txt
```

## Usage

1. Run the `gui.py` file to launch the application.
2. Enter the IP address of the UR5e robot and click "Connect to Robot".
3. Configure the motion parameters and click "Start Robot" to begin the movement.
4. Use the "Zero TCP" and "Zero Force" buttons as needed.
5. Data will be logged in real-time and can be exported to a CSV file after stopping the robot.

## License

This project is licensed under the MIT License.