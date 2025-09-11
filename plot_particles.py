import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSV
df = pd.read_csv("particles_log.csv")

# Separate types
particles = df[df['type'] == 'particle']
vehicle = df[df['type'] == 'vehicle']
landmark = df[df['type'] == 'landmark']

# Plot each timestep
timesteps = df['step'].unique()

for step in timesteps:
    plt.figure(figsize=(6, 6))
    step_particles = particles[particles['step'] == step]
    step_vehicle = vehicle[vehicle['step'] == step]
    step_landmark = landmark[landmark['step'] == step]

    # Plot particles
    plt.scatter(step_particles['x'], step_particles['y'], s=10, c='blue', label='Particles')

    # Plot vehicle
    plt.scatter(step_vehicle['x'], step_vehicle['y'], c='green', s=80, marker='o', label='Vehicle')
    
    # Plot vehicle orientation (arrow)
    vx, vy, theta = step_vehicle.iloc[0][['x','y','theta']]
    plt.arrow(vx, vy, 0.3*np.cos(theta), 0.3*np.sin(theta),
              head_width=0.1, head_length=0.1, fc='green', ec='green')

    # Plot landmark
    plt.scatter(step_landmark['x'], step_landmark['y'], c='red', s=80, marker='x', label='Landmark')

    plt.title(f"Particle Filter Step {step}")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.savefig(f"step_{step}.png")   # save each frame
    plt.close()

