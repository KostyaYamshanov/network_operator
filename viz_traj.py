import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
try:
    data = pd.read_csv("trajectories.csv")
except FileNotFoundError:
    print("Error: trajectories.csv not found. Please ensure the file exists.")
    exit(1)

# Verify columns
if not all(col in data.columns for col in ["Trajectory", "Time", "X", "Y", "Theta"]):
    print("Error: CSV file must contain columns 'Trajectory,Time,X,Y,Theta'.")
    exit(1)

# Colors for different trajectories
colors = ['blue', 'red', 'green', 'purple', 'orange', 'cyan', 'magenta', 'black']

# Plot each trajectory
plt.figure(figsize=(10, 8))
for i in range(8):  # 8 trajectories
    traj_data = data[data["Trajectory"] == i]
    plt.plot(traj_data["X"], traj_data["Y"], label=f"Trajectory {i}", color=colors[i], linewidth=2)
    # Mark start point
    plt.scatter(traj_data["X"].iloc[0], traj_data["Y"].iloc[0], color=colors[i], s=100, marker='o')
    # Mark end point
    plt.scatter(traj_data["X"].iloc[-1], traj_data["Y"].iloc[-1], color=colors[i], s=100, marker='x')

# Add goal point
plt.scatter(0, 0, color='red', s=200, marker='*', label='Goal (0,0)')

# Labels and title
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Trajectories to Goal (0,0,0)")
plt.legend()
plt.grid(True)

# Save and show plot
plt.savefig("trajectories_plot.png")
print("Plot saved as trajectories_plot.png")
plt.show()
