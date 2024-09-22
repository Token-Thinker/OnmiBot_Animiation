#Authored By: Michaël Guerrier (GitHub - Token Thinker)
# Imports
import numpy as np
from matplotlib import pyplot as plt, ticker
from matplotlib.patches import Rectangle, FancyArrow
from matplotlib.animation import FuncAnimation

# Global parameters for robot configuration
WHEEL_RADIUS = 0.148  # Radius of the omni wheels in meters
WHEEL_WIDTH = 0.044  # Width of the omni wheels in meters
ROBOT_RADIUS = 0.195  # Distance from the center of the robot platform to the wheels in meters
OMEGA = 0  # Default angular velocity (rad/s)
WHEEL_ANGLES = [np.radians(60), np.radians(180), np.radians(300)]  # Angles for a 3-wheel configuration
F_WHEEL_ANGLES = [np.radians(45), np.radians(135), np.radians(225), np.radians(315)]  # Angles for a 4-wheel configuration
PLOT_BOTH = False  # Flag to decide if both 3-wheel and 4-wheel configurations should be plotted
paused = False  # Flag to manage the animation's paused state

# Function to get user inputs or use default values
def get_user_inputs():
    """
    Gets user input for whether to plot both 3-wheel and 4-wheel configurations, and sets OMEGA.
    Returns:
        use_four_wheels: Boolean indicating whether to use 4 wheels (only asked if plotting a single config).
    """
    global OMEGA, PLOT_BOTH

    try:
        # Ask if the user wants to plot both configurations side by side
        PLOT_BOTH = input("Plot both 3-wheel and 4-wheel configurations? (y/n, default n): ").lower() == 'y'

        # Ask for OMEGA (angular velocity) value
        input_omega = float(input(f"Enter the omega value (default {OMEGA}): ") or OMEGA)
    except ValueError:
        print("Invalid input. Using default values.")
        input_omega = OMEGA
        PLOT_BOTH = False

    OMEGA = input_omega

    # If plotting both, don't ask about wheel configuration, otherwise ask if 4 wheels should be used
    if PLOT_BOTH:
        use_four_wheels = False  # Default when both are plotted
    else:
        use_four_wheels = input("Use 4 wheels? (y/n, default n): ").lower() == 'y'

    return use_four_wheels

# Function to construct the Jacobian matrix
def construct_jacobian(use_four_wheels=False):
    """
    Constructs the Jacobian matrix for the robot, determining the relationship between the robot's
    velocities and the wheel velocities based on the wheel configuration (3 or 4 wheels).

    Args:
        use_four_wheels: Boolean indicating whether the 4-wheel configuration is used.

    Returns:
        J: Jacobian matrix for the selected wheel configuration.
    """
    angles = F_WHEEL_ANGLES if use_four_wheels else WHEEL_ANGLES  # Use the appropriate wheel configuration
    j = np.zeros((len(angles), 3))  # Initialize a matrix with as many rows as there are wheels
    for i, angle in enumerate(angles):
        j[i, 0] = np.cos(angle) / WHEEL_RADIUS  # Contribution to x-velocity
        j[i, 1] = np.sin(angle) / WHEEL_RADIUS  # Contribution to y-velocity
        j[i, 2] = ROBOT_RADIUS / WHEEL_RADIUS   # Contribution to rotational velocity (omega)
    return j

# Function to convert robot speed and angle to body frame velocities
def convert_to_body_frame(speed, angle, orientation):
    """
    Converts the robot's speed and driving angle into body-frame velocities considering its orientation.

    Args:
        speed: Speed of the robot.
        angle: Driving angle of the robot (degrees).
        orientation: Orientation of the robot (degrees).

    Returns:
        v_bx, v_by: Body-frame x and y velocities.
    """
    angle_rad = np.radians(angle)  # Convert driving angle to radians
    orientation_rad = np.radians(orientation)  # Convert robot orientation to radians

    # Calculate the body-frame velocities before rotating them
    v_bx_0 = speed * np.cos(angle_rad - orientation_rad)  # Forward component
    v_by_0 = speed * np.sin(angle_rad - orientation_rad)  # Lateral component

    # Rotate 90 degrees to convert them to body-frame velocities
    v_bx = -v_by_0  # x-velocity (after rotation)
    v_by = v_bx_0   # y-velocity (after rotation)

    return v_bx, v_by

# Function to compute wheel velocities using the Jacobian model
def compute_wheel_velocities_jacobian(speed, angle, orientation, omega, use_four_wheels=False):
    """
    Computes the angular velocities of the wheels using the robot's velocity and the Jacobian matrix.

    Args:
        speed: Speed of the robot.
        angle: Driving angle of the robot.
        orientation: Orientation of the robot.
        omega: Angular velocity of the robot.
        use_four_wheels: Boolean indicating whether to use the 4-wheel configuration.

    Returns:
        wheel_velocities: Array of angular velocities for the wheels.
    """
    v_bx, v_by = convert_to_body_frame(speed, angle, orientation)  # Convert to body-frame velocities
    v = np.array([v_bx, v_by, omega])  # Create the velocity vector
    j = construct_jacobian(use_four_wheels)  # Get the Jacobian matrix
    wheel_velocities = np.dot(j, v)  # Calculate the wheel velocities
    return wheel_velocities

# Function to plot the robot and wheels on the provided axis
def plot_robot(ax, wheel_velocities, orientation, speed, angle, use_four_wheels=False):
    """
    Plots the robot's body, wheels, and velocity vectors based on the wheel velocities and orientation.

    Args:
        ax: The axis on which to plot the robot.
        wheel_velocities: Angular velocities of the wheels.
        orientation: Robot's orientation.
        speed: Robot's speed.
        angle: Driving angle.
        use_four_wheels: Boolean indicating whether the 4-wheel configuration is used.
    """
    ax.clear()  # Clear the axis for redrawing

    # Plot the robot's body as a circle
    robot_circle = plt.Circle((0, 0), ROBOT_RADIUS, edgecolor='black', facecolor='blue', fill=True, lw=2, alpha=0.5)
    ax.add_patch(robot_circle)

    # Calculate the robot's forward position for orientation
    orientation_rad = np.radians(orientation)
    forward_position_x = ROBOT_RADIUS * np.cos(orientation_rad)
    forward_position_y = ROBOT_RADIUS * np.sin(orientation_rad)
    ax.plot(forward_position_x, forward_position_y, 'bo', label="Forward Position")

    # Plot the direction arrow based on speed
    if speed > 0.01:  # Only plot if speed is significant
        sp_arrow_max = ROBOT_RADIUS * 0.90  # Maximum arrow length
        sp_scaled = sp_arrow_max * speed
        end_x = round(sp_scaled * np.cos(np.radians(angle)), 4)
        end_y = round(sp_scaled * np.sin(np.radians(angle)), 4)
        sp_arrow = FancyArrow(0, 0, end_x, end_y, width=0.01, color='white', length_includes_head=False, head_width=.02, head_length=.02)
        ax.add_patch(sp_arrow)

    # Plot wheels and wheel velocity vectors
    wheel_angles = F_WHEEL_ANGLES if use_four_wheels else WHEEL_ANGLES
    for i, w_angle_rad in enumerate(wheel_angles):
        rotated_angle_rad = w_angle_rad + orientation_rad
        x = ROBOT_RADIUS * np.cos(rotated_angle_rad)
        y = ROBOT_RADIUS * np.sin(rotated_angle_rad)

        # Calculate rectangle (wheel) position
        rect_x = x + ((WHEEL_WIDTH / 2) * np.cos(rotated_angle_rad)) - ((WHEEL_RADIUS / 2) * np.cos(rotated_angle_rad + np.pi / 2))
        rect_y = y + ((WHEEL_WIDTH / 2) * np.sin(rotated_angle_rad)) - ((WHEEL_RADIUS / 2) * np.sin(rotated_angle_rad + np.pi / 2))
        wheel = Rectangle((rect_x, rect_y), WHEEL_RADIUS, WHEEL_WIDTH, angle=np.degrees(rotated_angle_rad + np.pi/2), ec='black', fc='white', fill=True)
        ax.add_patch(wheel)

        #Calculate rectangle buffer for velocity arrows
        buffer_x = x + ((WHEEL_WIDTH / 2 + 0.01) * np.cos(rotated_angle_rad)) - ((WHEEL_RADIUS / 2 + 0.01) * np.cos(rotated_angle_rad + np.pi / 2))
        buffer_y = y + ((WHEEL_WIDTH / 2 + 0.01) * np.sin(rotated_angle_rad)) - ((WHEEL_RADIUS / 2 + 0.01) * np.sin(rotated_angle_rad + np.pi / 2))

        # Plot velocity vectors for each wheel
        velocity_magnitude = wheel_velocities[i]
        if abs(velocity_magnitude) >= 0.1:  # Only plot if velocity is significant
            vm_x = buffer_x + (WHEEL_RADIUS  / 2) * np.cos(rotated_angle_rad + np.pi / 2)
            vm_y = buffer_y + (WHEEL_RADIUS / 2) * np.sin(rotated_angle_rad + np.pi / 2)

            v_arrow_max = WHEEL_RADIUS * 0.3  # Set the maximum arrow length
            max_velocity = 6  # Max velocity for current robot parameters to normalize arrow length

            # Calculate the velocity vector direction and magnitude
            vm_scaled = (abs(velocity_magnitude) / max_velocity) * v_arrow_max

            # Adjust the direction of the arrow based on the wheel velocity
            if velocity_magnitude < 0:
                # Flip the arrow direction for negative velocities
                sv_x = vm_scaled * np.cos(rotated_angle_rad + np.pi / 2)
                sv_y = vm_scaled * np.sin(rotated_angle_rad + np.pi / 2)
            else:
                # Normal direction for positive velocities
                sv_x = -vm_scaled * np.cos(rotated_angle_rad + np.pi / 2)
                sv_y = -vm_scaled * np.sin(rotated_angle_rad + np.pi / 2)

            # Plot the velocity arrow for the wheel
            ax.arrow(
                vm_x, vm_y,
                sv_x, sv_y,
                head_width=0.01, head_length=0.01, fc='blue', ec='blue', lw=1
            )

        # Label the wheels with their index
        ax.text(x, y, str(i), fontsize=12, ha='center', va='center', color='black')

    # Set axis limits and ensure aspect ratio
    ax.set_aspect('equal')
    ax.set_xlim([-0.4, 0.4])
    ax.set_ylim([-0.3, 0.4])

    # Set grid and ticks
    ax.xaxis.set_major_locator(ticker.MultipleLocator(0.1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.1))
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda val, pos: '{:.0f}'.format(val * 10)))
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda val, pos: '{:.0f}'.format(val * 10)))

    # Remove grid lines
    plt.grid(False)

    # Call plot_text to display velocity and robot info on the plot
    plot_text(ax, speed, angle, orientation, wheel_velocities, use_four_wheels)

# Function to plot text information
def plot_text(ax, speed, angle, orientation, wheel_velocities, use_four_wheels):
    """
    Plots the text information box showing speed, direction, orientation, and wheel velocities.
    Dynamically sets the title based on whether the robot uses 3 wheels or 4 wheels.
    """
    # Construct omega string for all wheels
    w_omega_text = ', '.join([f'{v:.1f}' for v in wheel_velocities])

    # Info box with robot's current state
    info_text = (
        f"robot orient.={orientation:.1f}°\n"
        f"driving dir={angle:.1f}°\n"
        f"driving speed (m/s) ={speed:.1f}\n"
        f"ω (rad/s) = [{w_omega_text}]"
    )

    # Display the info box in the top-left corner of the axis
    ax.text(
        0.05, .95,
        info_text,
        fontsize=12,
        bbox=dict(fc='azure', alpha=0.5),
        ha='left', va='top',
        transform=ax.transAxes
    )

    # Set plot title based on wheel configuration
    title_text = 'Jacobian Omnidirectional - 4 Wheels' if use_four_wheels else 'Jacobian Omnidirectional - 3 Wheels'
    ax.set_title(title_text, fontsize=16, fontweight='bold')

# Toggle pause function
def toggle_pause(event):
    global paused
    if event.key == ' ':
        paused = not paused

# Update function for animation frames
def update(frame, ax, omega, use_four_wheels=False):
    """
    Updates the animation by recalculating wheel velocities and plotting the robot's state.
    The animation can be paused and resumed by toggling the 'paused' state.
    """
    if not paused:
        ax.clear()

        # Oscillate speed between 0 and 1 based on the frame number
        speed = 0.5 * (1 + np.sin(np.radians(frame)))

        # Increment the driving angle from 0 to 360 degrees
        angle = frame % 360

        # Change the robot's orientation based on speed and omega
        orientation_change_rate = omega * frame
        orientation = orientation_change_rate % 360

        # Compute the wheel velocities using the Jacobian model
        wheel_velocities = compute_wheel_velocities_jacobian(speed, angle, orientation, omega, use_four_wheels)

        # Update the plot with the new values
        plot_robot(ax, wheel_velocities, orientation, speed, angle, use_four_wheels)

# Main function to run the animation
def run_animation():
    """
    Main function to run the animation. It allows for plotting either a single configuration (3-wheel or 4-wheel)
    or both configurations side by side.
    """
    use_four_wheels = get_user_inputs()

    if PLOT_BOTH:
        # Create a figure with two subplots for side-by-side comparison of 3-wheel and 4-wheel configurations
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

        # 3-wheel configuration on the left
        ani1 = FuncAnimation(fig, update, fargs=(ax1, OMEGA, False), frames=np.arange(0, 720, 1), interval=50, repeat=True)

        # 4-wheel configuration on the right
        ani2 = FuncAnimation(fig, update, fargs=(ax2, OMEGA, True), frames=np.arange(0, 720, 1), interval=50, repeat=True)
    else:
        # Create a single plot for either 3 or 4 wheels
        fig, ax = plt.subplots(figsize=(8, 8))

        # Animate the selected configuration
        ani = FuncAnimation(fig, update, fargs=(ax, OMEGA, use_four_wheels), frames=np.arange(0, 720, 1), interval=50, repeat=True)

    # Connect the pause functionality to the figure (spacebar to pause)
    fig.canvas.mpl_connect('key_press_event', toggle_pause)

    # Show the plot
    plt.show()

# Run the animation
run_animation()