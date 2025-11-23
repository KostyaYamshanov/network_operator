import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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

# ✨ НОВОЕ: Динамически определяем количество траекторий из CSV
trajectory_ids = sorted(data["Trajectory"].unique())
num_trajectories = len(trajectory_ids)
print(f"Found {num_trajectories} trajectories in CSV")

# ✨ DEBUG: Выводим начальные точки каждой траектории
print("\n=== START POINTS ===")
for traj_id in trajectory_ids:
    traj_data = data[data["Trajectory"] == traj_id]
    # Получаем первую точку (начало траектории)
    first_row = traj_data.iloc[0]
    print(f"Trajectory {traj_id}: Start ({first_row['X']:.3f}, {first_row['Y']:.3f}, {first_row['Theta']:.3f})")

# Генерируем цвета для каждой траектории (больше чем 8)
colors = plt.cm.tab20(np.linspace(0, 1, num_trajectories))
if num_trajectories > 20:
    # Если траекторий больше 20, используем другую color map
    colors = plt.cm.hsv(np.linspace(0, 1, num_trajectories))

# Plot each trajectory
plt.figure(figsize=(12, 10))
for i, traj_id in enumerate(trajectory_ids):
    traj_data = data[data["Trajectory"] == traj_id]
    
    # Получаем начальное и конечное состояния
    start_x = traj_data["X"].iloc[0]
    start_y = traj_data["Y"].iloc[0]
    end_x = traj_data["X"].iloc[-1]
    end_y = traj_data["Y"].iloc[-1]
    
    # Рисуем траекторию
    plt.plot(traj_data["X"], traj_data["Y"], 
             label=f"Traj {traj_id}", 
             color=colors[i], 
             linewidth=2, 
             alpha=0.8)
    
    # Отмечаем начальную точку (зелёный круг)
    plt.scatter(start_x, start_y, color=colors[i], s=100, marker='o', edgecolors='black', linewidth=1)
    
    # Отмечаем конечную точку (красный крест)
    plt.scatter(end_x, end_y, color=colors[i], s=100, marker='x', linewidth=2)

# Add goal point (красная звезда)
plt.scatter(0, 0, color='red', s=300, marker='*', label='Goal (0,0)', edgecolors='black', linewidth=1, zorder=10)

# Labels and title
plt.xlabel("X (m)", fontsize=12)
plt.ylabel("Y (m)", fontsize=12)
plt.title(f"Robot Trajectories to Goal (0,0,0) - {num_trajectories} trajectories", fontsize=14)

# Улучшенная легенда (показываем только каждую 4-ю если траекторий много)
if num_trajectories <= 20:
    plt.legend(loc='best', fontsize=8, ncol=2)
else:
    # Для большого количества траекторий показываем только часть легенды
    handles, labels = plt.gca().get_legend_handles_labels()
    plt.legend(handles[::4] + [handles[-1]], labels[::4] + [labels[-1]], loc='best', fontsize=8)

plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()

# Save and show plot
plt.savefig("trajectories_plot.png", dpi=150)
print(f"\nPlot saved as trajectories_plot.png")
print(f"✓ {num_trajectories} trajectories plotted")
print(f"  - Green circles: start points")
print(f"  - Red crosses: end points")
print(f"  - Red star: goal (0,0)")
plt.show()