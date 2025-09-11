import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('particles_log.csv')

unique_steps = sorted(df['step'].unique())
for time_step in unique_steps:
    step_data = df[df['step'] == time_step]
    particles = step_data[step_data['type'] == 'particle']
    particle_landmarks = step_data[step_data['type'] == 'particle_landmark']
    true_landmark = step_data[step_data['type'] == 'true_landmark']
    vehicle = step_data[step_data['type'] == 'vehicle']

    plt.figure(figsize=(10, 8))
    plt.scatter(particles['x'], particles['y'], s=10, c='blue', label='Particles')

    for _, row in particles.iterrows():
        if 'theta' in row:
            plt.arrow(row['x'], row['y'],
                      0.5 * np.cos(row['theta']),
                      0.5 * np.sin(row['theta']),
                      head_width=0.1, head_length=0.1, fc='blue', ec='blue', alpha=0.5)

    if not particle_landmarks.empty:
        plt.scatter(particle_landmarks['x'], particle_landmarks['y'], c='green', marker='o', s=60, label='Estimated Landmark')
    if not true_landmark.empty:
        plt.scatter(true_landmark['x'], true_landmark['y'], c='red', marker='*', s=200, label='True Landmark')
    if not vehicle.empty:
        plt.scatter(vehicle['x'], vehicle['y'], c='magenta', marker='s', s=120, label='True Vehicle')

    plt.title(f'Particles, Landmarks, and Orientations at Time Step {time_step}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.show()