# Unpowered Glider Simulation

This project simulates the flight of a glider, controlled by a user-defined controller. The simulation environment is built in MATLAB, providing a platform for designing, implementing, and testing control strategies for this aerial vehicle.

## Overview

The `DesignProblem03.m` script serves as the main entry point for running the glider simulation. It requires the name of a MATLAB function that defines the control logic as its primary input. Additionally, it accepts several optional parameters to customize the simulation behavior, including team identification, data logging, movie and snapshot saving, defining a reference trajectory, setting initial conditions, and controlling the display.

The simulation models the 2D dynamics of a glider, tracking its position (x, y), orientation (theta, phi), and velocities (xdot, ydot, thetadot). The user-defined controller function receives sensor data (current glider state) and a reference value for the pitch angle (theta) and outputs a commanded elevator angular velocity (phidot).

**Key Features:**

* **Customizable Controller:** Users can implement their own control algorithms by creating a MATLAB function with `init` and `run` sub-functions.
* **Simulation Parameter Tuning:** Various optional parameters allow for flexible experimentation with different simulation scenarios.
* **Data Logging:** Simulation data can be saved to a `.mat` file for subsequent analysis.
* **Visualization:** A graphical representation of the glider's flight is displayed during the simulation.
* **Reference Tracking:** The controller aims to make the glider's pitch angle follow a time-varying reference trajectory defined by a user-provided function.
* **Initial Condition Flexibility:** Users can specify the initial state of the glider, including position, orientation, and velocities.
* **Launch Angle Control:** An option to set the mean launch angle for the initial conditions.
* **Elevator Length Adjustment:** The length of the elevator control surface can be modified.

## Getting Started

1.  **MATLAB Environment:** Ensure you have MATLAB installed.
2.  **Controller Function:** Create a MATLAB function file (e.g., `MyGliderController.m`) that contains your control logic. This file must define two sub-functions: `initControlSystem` and `runControlSystem`, adhering to the input and output specifications detailed within `DesignProblem03.m`.
3.  **Run the Simulation:** Execute the `DesignProblem03.m` script from the MATLAB command window, providing the name of your controller function as the first argument.

    ```matlab
    DesignProblem03('MyGliderController');
    ```

## Controller Interface

Your controller function must define the following sub-functions:

* **`initControlSystem(parameters, data)`:** This function is called once at the beginning of the simulation. It's used to initialize controller parameters and internal data structures.
    * **Inputs:**
        * `parameters`: A struct containing physical constants and simulation settings (e.g., `tStep`, `phidotMax`, `symEOM`, `numEOM`).
        * `data`: A struct for storing controller-specific data.
    * **Output:**
        * `data`: The (potentially modified) data struct.

* **`runControlSystem(sensors, references, parameters, data)`:** This function is called at each time step of the simulation. It implements your control algorithm.
    * **Inputs:**
        * `sensors`: A struct containing the current state of the glider (e.g., `t`, `theta`, `phi`).
        * `references`: A struct containing the desired reference value for the pitch angle (`theta`).
        * `parameters`: The same parameters struct passed to `initControlSystem`.
        * `data`: The data struct updated in `initControlSystem` and previous calls to `runControlSystem`.
    * **Outputs:**
        * `actuators`: A struct containing the commanded elevator angular velocity (`phidot`).
        * `data`: The (potentially modified) data struct for use in the next time step.

## Optional Parameters

You can customize the simulation using parameter-value pairs when calling `DesignProblem03`:

* `'team'`: A string specifying a team name to display on the figure window.
* `'datafile'`: A string specifying the filename for saving simulation data (e.g., `'glider_data.mat'`).
* `'moviefile'`: A string specifying the filename for saving a movie of the simulation (e.g., `'glider_movie.mp4'`).
* `'snapshotfile'`: A string specifying the filename for saving a PDF snapshot of the final simulation frame (e.g., `'glider_snap.pdf'`).
* `'controllerdatatolog'`: A cell array of strings specifying fields in `controller.data` to log in the data file (if `'datafile'` is defined).
* `'tStop'`: A positive scalar number specifying the simulation stop time (default is `30`).
* `'reference'`: A function handle that takes time as input and returns the desired pitch angle (e.g., `@(t) 0.2*sin(t)`). The default is `@(t)0`.
* `'initial'`: A 7x1 numerical matrix `[x; y; theta; phi; xdot; ydot; thetadot]` specifying the initial state of the glider. The default is sampled from a multivariate normal distribution.
* `'launchangle'`: An angle in radians that will be the mean of the normal distributions from which the initial pitch angle is sampled (default is `0`).
* `'elevatorlen'`: A number between `0` and `0.2` specifying the length of the elevator (default is `0.05`).
* `'display'`: A logical flag (`true` or `false`) to enable or disable the live simulation display (default is `true`).
* `'seed'`: A non-negative integer to seed the random number generator for reproducible initial conditions.

**Example with optional parameters:**

```matlab
DesignProblem03('MyGliderController', 'team', 'AirTeam', 'datafile', 'flight_log.mat', 'tStop', 15, 'reference', @(t) 0.1*cos(t), 'initial', [0; 1; deg2rad(5); deg2rad(2); 5; 0; 0], 'elevatorlen', 0.1);
```
## Understanding the Code

The `DesignProblem03.m` script orchestrates the glider simulation. Here's a breakdown of its key functions:

* **`SetupSimulation(process)`:**
    * Initializes the simulation environment, including setting the random number generator seed for reproducibility.
    * Defines the physical constants of the glider, such as mass, gravity, air density, dimensions, and moment of inertia.
    * Loads or computes the symbolic (`symEOM`) and numeric (`numEOM`) equations of motion that govern the glider's dynamics.
    * Sets up the user-defined controller by:
        * Obtaining handles to the `initControlSystem` and `runControlSystem` functions.
        * Creating a `parameters` structure containing relevant simulation constants.
        * Initializing a `data` structure for the controller's internal use.

* **`RunController(controller)`:**
    * Executes the `runControlSystem` function of the user-provided controller.
    * Passes the current sensor readings (`sensors`), reference values (`references`), simulation parameters (`parameters`), and the controller's internal data (`data`) as inputs.
    * Receives the commanded elevator angular velocity (`actuators.phidot`) and the potentially updated controller data (`data`).
    * Includes error handling to gracefully manage exceptions that might occur within the controller's `run` function.

* **`GetReferences(process)`:**
    * Evaluates the user-defined reference function (specified via the `'reference'` parameter) at the current simulation time (`process.t`).
    * Returns a structure containing the desired pitch angle (`references.theta`).
    * Includes error handling for potential issues with the reference function.

* **`GetSensors(process)`:**
    * Extracts the relevant state variables of the glider from the `process` structure to provide feedback to the controller. These typically include the current time (`sensors.t`), pitch angle (`sensors.theta`), and elevator angle (`sensors.phi`).

* **`Get_TandX_From_Process(process)`:**
    * A utility function that takes the `process` structure and returns the current simulation time (`t`) and the state vector (`x = [x; y; theta; phi; xdot; ydot; thetadot]`) in a format suitable for the `ode45` integrator.

* **`GetInput(process, actuators)`:**
    * Takes the commanded elevator angular velocity (`actuators.phidot`) from the controller.
    * Applies saturation to the commanded value, ensuring it stays within the physical limits defined by `process.phidotMax`.

* **`Get_Process_From_TandX(t, x, process)`:**
    * A utility function that takes the time (`t`) and state vector (`x`) returned by the `ode45` integrator and updates the corresponding fields in the `process` structure.

* **`GetXDot(t, x, u, process)`:**
    * Defines the continuous-time dynamics of the glider. It takes the current time (`t`), state vector (`x`), control input (`u` - the commanded elevator angular velocity), and the `process` structure as inputs.
    * Calculates the time derivative of the state vector (`xdot`) using the pre-computed numeric equations of motion (`process.numEOM`).

* **`CheckActuators(actuators)`:**
    * A validation function that checks if the `actuators` structure returned by the controller has the expected format, specifically ensuring it contains a scalar numeric field named `phidot`.

* **`ZeroActuators()`:**
    * Returns a default `actuators` structure with the elevator angular velocity (`phidot`) set to zero. This is used as a safe default if the controller encounters an error.

* **`ShouldStop(process)`:**
    * Defines the condition under which the simulation should terminate. In this case, the simulation stops if the glider's vertical position (`process.y`) becomes less than or equal to zero (indicating a ground landing).

* **`UpdateFigure(process, controller, fig)`:**
    * Manages the graphical visualization of the glider's flight.
    * Creates a new figure or updates an existing one to display the glider's position, orientation, and the trajectory it has followed.
    * Shows relevant information such as the simulation time, horizontal distance traveled, and the status of the controller (ON or OFF).
    * Implements a key press function (`onkeypress`) to allow the user to stop the simulation prematurely by pressing the 'q' key.
    * Uses the `DrawBox` helper function to render the glider's components.

* **`DrawBox(box, x, y, theta, d)`:**
    * A helper function that draws a rotated rectangle representing a part of the glider (spar, wing, elevator). It takes the current graphics object (`box`), the center coordinates (`x`, `y`), the rotation angle (`theta`), and the dimensions (`d`) of the rectangle as input.

* **`RunSimulation(process, controller)`:**
    * The main loop of the simulation.
    * Initializes the figure and sets up movie recording if a `moviefile` is specified.
    * Iteratively:
        * Updates the graphical display using `UpdateFigure`.
        * Logs simulation data using `UpdateDatalog`.
        * Records frames for a movie if enabled.
        * Checks if the stopping condition (`ShouldStop` or user interrupt) has been met.
        * Integrates the glider's dynamics for one time step using `ode45` in `UpdateProcess`.
        * Pauses to maintain real-time simulation if the display is active.
    * Handles the shutdown process by closing the movie file, saving the logged data to a `.mat` file, and saving a snapshot of the final frame as a PDF if requested.

* **`UpdateDatalog(process, controller)`:**
    * Manages the logging of simulation data.
    * Creates a `log` structure within the `process` if it doesn't exist.
    * Records process-related data (time, states) and controller-related data (initialization time, run time, sensor values, actuator commands, and optionally user-defined controller data specified by `controllerdatatolog`).

* **`UpdateProcess(process, controller)`:**
    * Performs one step of the simulation.
    * Obtains the current time and state from the `process`.
    * Gets the control input from the controller using `GetInput`.
    * Integrates the equations of motion over one time step using `ode45` with the `GetXDot` function.
    * Updates the `process` structure with the new time and state.
    * Gets the reference values for the next control step using `GetReferences`.
    * Gets the sensor readings for the next control step using `GetSensors`.
    * Runs the controller to obtain the actuator commands for the next time step using `RunController`.

## Included Test Code

The script includes a section at the end that provides an example of a simple controller (`Controller` function) and demonstrates how to run the simulation with it. This test code typically illustrates how to:

* Define a basic controller structure with `initControlSystem` and `runControlSystem` sub-functions.
* Call `DesignProblem03` with the name of the example controller.
* Use optional parameters like `'launchangle'` and `'elevatorlen'` to configure the simulation.
* Run the simulation multiple times (e.g., in a loop) to collect data on the glider's performance, such as the final horizontal distance traveled.
* Generate plots, such as the flight paths and histograms of the distances flown, to visualize the results.
* Perform basic data analysis, such as calculating the average and median distances and identifying potential outliers.

By studying and executing this test code, users can gain a practical understanding of how to create their own controllers and interact with the glider simulation environment. The provided analysis also serves as a starting point for evaluating the effectiveness of different control strategies.
