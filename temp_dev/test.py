import matplotlib.pyplot as plt
import numpy as np

# Define curvature values
max_curvatures = [0.2, 0.5, 1.0, 2.0]  # Example values of max_curvature
colors = ['red', 'blue', 'green', 'orange']

# Plot settings
plt.figure(figsize=(8, 8))
plt.title("Relationship Between Max Curvature and Radius", fontsize=14)
plt.xlabel("X (meters)", fontsize=12)
plt.ylabel("Y (meters)", fontsize=12)
plt.grid(True)
plt.axis('equal')

# Draw each curve
for i, max_curvature in enumerate(max_curvatures):
    radius = 1 / max_curvature
    theta = np.linspace(0, np.pi / 2, 100)  # Quarter circle
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    
    label = f"Curvature: {max_curvature:.2f}, Radius: {radius:.2f} m"
    plt.plot(x, y, label=label, color=colors[i])

# Add legend
plt.legend(fontsize=10)
plt.show()
