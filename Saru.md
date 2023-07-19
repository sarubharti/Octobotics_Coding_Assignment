# Octobotics Coding Assignment

### Dependencies

- [`control`]

```bash
pip install pygame
```

```bash
source devel/setup.bash
```

- launch the sinusoidal force simulation and plotting using roslaunch

```bash
roslaunch inverted_pendulum_sim inverted_pendulum_sim.launch
```

- launch the PID control force simulation and plotting using roslaunch

```bash
roslaunch inverted_pendulum_sim control.launch
```

### Published Topics
- /inverted_pendulum/current_state ([inverted_pendulum_sim/CurrentState](https://github.com/octobotics/Octobotics_Coding_Assignment/blob/main/src/inverted_pendulum_sim/msg/CurrentState.msg)) - Publishes the current state of the inverted pendulum at 100 Hz
 
### Subscribed Topics
- /inverted_pendulum/control_force ([inverted_pendulum_sim/ControlForce](https://github.com/octobotics/Octobotics_Coding_Assignment/blob/main/src/inverted_pendulum_sim/msg/ControlForce.msg)) - Subscribes to the control force input to the inverted pendulum

### Services
- /inverted_pendulum/set_params ([inverted_pendulum_sim/SetParams](https://github.com/octobotics/Octobotics_Coding_Assignment/tree/main/src/inverted_pendulum_sim/src) - Sets the parameters and initial conditions of the inverted pendulum
