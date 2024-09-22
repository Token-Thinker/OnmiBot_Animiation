## Simplified Jacobian Omnidirectional Robot Simulation

This project simulates an omnidirectional robot using a simplified version of the Jacobian matrix model.

![Robot Simulation](robot_simulation.gif)


## Background
The **Jacobian matrix** \( J \) relates the body-frame velocities to the individual wheel velocities. 

For a **3-wheel configuration**, the matrix is:

$$ J = 
\begin{bmatrix}
\cos(\theta_1) / r & \sin(\theta_1) / r & L / r \\
\cos(\theta_2) / r & \sin(\theta_2) / r & L / r \\
\cos(\theta_3) / r & \sin(\theta_3) / r & L / r \\
\end{bmatrix}
$$

Where:
- ($\theta_1$, $\theta_2$, $\theta_3$) are the angles of the wheels
- \($r$) is the wheel radius
- \($L$) is the distance from the robot’s center to the wheels

The wheel velocities \(\mathbf{\Omega}\) are calculated using:

$$ \mathbf{\Omega} = J \mathbf{V} $$

where $$\mathbf{V} = [v_x, v_y, \omega]^T$$ represents the body-frame velocities.



## Features

- **3-Wheel and 4-Wheel Configurations:** Choose between 3-wheel and 4-wheel setups or display both configurations side by side for comparison.
- **Velocity Arrows:** Visualize the velocity of each wheel dynamically calculated based on robot motion.
- **Pause/Resume Functionality:** Pause and resume the simulation using the spacebar.
- **Customizable Omega:** The robot's angular velocity (omega) can be customized through user input.

## Getting Started

### Prerequisites

Ensure you have the following Python packages installed:
`pip install numpy matplotlib`


### Running the Simulation

To run the simulation, execute the Python script:
`python jacobian_simulation.py`


### User Inputs

Upon running the script, you'll be prompted for the following:

1. **Plot both configurations?** (`y/n`):
   - Choose whether to display both 3-wheel and 4-wheel configurations side by side.
   
2. **Omega (angular velocity)**:
   - Enter the desired angular velocity (omega). If left blank, the default value will be used.
   
3. **Use 4 wheels?** (`y/n`):
   - If not plotting both configurations, you'll be asked if you want to use the 4-wheel configuration.

## Credits
This project was created by [Michaël Guerrier (Token Thinker)](https://github.com/Token-Thinker).

The Jacobian matrix calculations for omnidirectional platforms are based on the article: 
[**"Defining the Consistent Velocity of Omnidirectional Mobile Platforms"**](https://www.mdpi.com/2075-1702/12/6/397) published by MDPI.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
