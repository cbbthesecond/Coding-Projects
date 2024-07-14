import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime, timedelta

class Body:
    def __init__(self, name, mass, position, velocity, color):
        #General Body Inits
        self.name = name
        self.mass = mass
        self.position = np.array(position, dtype='float64')
        self.velocity = np.array(velocity, dtype='float64')
        self.force = np.zeros(3)
        self.color = color
        self.initial_position = np.array(position, dtype='float64')
        self.initial_distance = np.linalg.norm(position)

        #Orbit Tracing Inits
        self.positions = [self.position.copy()]  #Stores Position Vector for Single Orbit
        self.orbit_complete = False #Orbit Completion Flag

def compute_force(b1, b2, G=6.67430e-11):
    r = b2.position - b1.position
    distance = np.linalg.norm(r)
    if distance == 0:
        return np.zeros(3)
    force_magnitude = G * b1.mass * b2.mass / distance**2
    force_direction = r / distance
    return force_magnitude * force_direction

def update_body(body, dt):
    body.velocity += body.force / body.mass * dt
    body.position += body.velocity * dt
    body.positions.append(body.position.copy())
    if not body.orbit_complete:
        current_distance = np.linalg.norm(body.position - body.initial_position)
        if current_distance < body.initial_distance * 0.01:  # Within 1% of the initial distance (can change this for higher/lower accuracy)
            body.orbit_complete = True #Change Value of Completion Flag 

def simulate(bodies, dt, iterations_per_frame): #The cool stuff (imo)
    start_date = datetime(2024, 1, 1) #Sim Starts Jan 1, 2024
    days_passed = 0
    while True: #Infinite Run Time, can put a var Flag in here and add a time constraint if desired
        for _ in range(iterations_per_frame):
            for body in bodies:
                body.force[:] = 0
                for other_body in bodies:
                    if body is not other_body:
                        body.force += compute_force(body, other_body)
                update_body(body, dt)
            days_passed += dt / 86400  # Convert seconds to days
        current_date = start_date + timedelta(days=days_passed)
        yield bodies, current_date

def shoot_asteroids(bodies, n):
    for i in range(n):
        mass = np.random.uniform(1e8, 1e15)  # Random mass for the asteroid

        # Ensure position is within a range to cross the solar system
        position = np.random.uniform(-5e12, 5e12, 3)
        position[np.random.randint(0, 3)] *= np.random.choice([-1, 1])

        # Reduce velocity to increase the chance of getting caught in the solar system's gravity
        velocity = np.random.uniform(-1000, 1000, 3)
        asteroid = Body(f"Asteroid_{i+1}", mass, position, velocity, 'gray')
        bodies.append(asteroid)

def plot_positions(bodies, positions_generator):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set plot limits to center the Sun
    max_range = 5e12
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)

    # Set a dark background color to resemble space
    ax.set_facecolor('#1a1a1a')
    fig.patch.set_facecolor('#1a1a1a')
    ax.w_xaxis.set_pane_color((0.1, 0.1, 0.1, 1))
    ax.w_yaxis.set_pane_color((0.1, 0.1, 0.1, 1))
    ax.w_zaxis.set_pane_color((0.1, 0.1, 0.1, 1))
    ax.grid(False)
    ax.xaxis.set_tick_params(color='white')
    ax.yaxis.set_tick_params(color='white')
    ax.zaxis.set_tick_params(color='white')
    ax.tick_params(axis='both', which='major', labelcolor='white')

    # Add stars in the background
    num_stars = 100
    star_x = np.random.uniform(-max_range, max_range, num_stars)
    star_y = np.random.uniform(-max_range, max_range, num_stars)
    star_z = np.random.uniform(-max_range, max_range, num_stars)
    ax.scatter(star_x, star_y, star_z, color='white', s=1)

    # Plot initial positions and lines for orbits
    lines = []
    orbits = []
    for i, body in enumerate(bodies):
        line, = ax.plot([], [], [], 'o', color=body.color, markersize=10 if body.name == "Sun" else 5, label=body.name)
        orbit, = ax.plot([], [], [], '-', color=body.color, linewidth=0.5)
        lines.append(line)
        orbits.append(orbit)

    # Add date and century counter text
    date_text = ax.text2D(0.85, 0.95, '', transform=ax.transAxes, color='white')
    century_text = ax.text2D(0.85, 0.92, '', transform=ax.transAxes, color='white')

    def update(frame):
        bodies, current_date = next(positions_generator)
        centuries_passed = (current_date.year - 2024) / 100
        for i, (line, orbit) in enumerate(zip(lines, orbits)):
            x, y, z = bodies[i].position
            line.set_data([x], [y])
            line.set_3d_properties([z])
            orbit.set_data([p[0] for p in bodies[i].positions], [p[1] for p in bodies[i].positions])
            orbit.set_3d_properties([p[2] for p in bodies[i].positions])
        date_text.set_text(f'Date: {current_date.strftime("%Y-%m-%d")}')
        century_text.set_text(f'Centuries: {centuries_passed:.2f}')
        return lines + orbits + [date_text, century_text]

    def on_key(event): #Allows for Zoom In/Out with +/- keys (no shift)
        scale_factor = 1.1
        if event.key == '=':
            ax.set_xlim(ax.get_xlim()[0] / scale_factor, ax.get_xlim()[1] / scale_factor)
            ax.set_ylim(ax.get_ylim()[0] / scale_factor, ax.get_ylim()[1] / scale_factor)
            ax.set_zlim(ax.get_zlim()[0] / scale_factor, ax.get_zlim()[1] / scale_factor)
        elif event.key == '-':
            ax.set_xlim(ax.get_xlim()[0] * scale_factor, ax.get_xlim()[1] * scale_factor)
            ax.set_ylim(ax.get_ylim()[0] * scale_factor, ax.get_ylim()[1] * scale_factor)
            ax.set_zlim(ax.get_zlim()[0] * scale_factor, ax.get_zlim()[1] * scale_factor)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', on_key)

    ani = FuncAnimation(fig, update, frames=positions_generator, interval=50, blit=True, cache_frame_data=False)
    
    # Creates legend
    legend = ax.legend(loc='upper left', fontsize='small', facecolor='black', edgecolor='white')
    for text in legend.get_texts():
        text.set_color('white')

    plt.show()

# Example usage:
bodies = [
    Body("Sun", 1.989e30, [0, 0, 0], [0, 0, 0], 'yellow'),  # Sun
    Body("Mercury", 3.3011e23, [5.79e10, 0, 0], [0, 47400, 0], 'gray'),  # Mercury
    Body("Venus", 4.8675e24, [1.082e11, 0, 0], [0, 35020, 0], 'orange'),  # Venus
    Body("Earth", 5.972e24, [1.496e11, 0, 0], [0, 29783, 0], 'blue'),  # Earth
    Body("Mars", 6.4171e23, [2.279e11, 0, 0], [0, 24077, 0], 'red'),  # Mars
    Body("Jupiter", 1.8982e27, [7.785e11, 0, 0], [0, 13070, 0], 'brown'),  # Jupiter
    Body("Saturn", 5.6834e26, [1.433e12, 0, 0], [0, 9690, 0], 'gold'),  # Saturn
    Body("Uranus", 8.6810e25, [2.872e12, 0, 0], [0, 6800, 0], 'cyan'),  # Uranus
    Body("Neptune", 1.02413e26, [4.495e12, 0, 0], [0, 5430, 0], 'darkblue')   # Neptune
]

# Number of asteroids to shoot into the system
n_asteroids = 5
shoot_asteroids(bodies, n_asteroids)

positions_generator = simulate(bodies, 86400, 5)  # One day per step, 5 iterations per frame
plot_positions(bodies, positions_generator)
