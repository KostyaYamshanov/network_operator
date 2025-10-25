import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
try:
    data = pd.read_csv("function_data.csv")
except FileNotFoundError:
    print("Error: function_data.csv not found. Please ensure the file exists in the working directory.")
    exit(1)

# Verify that the required columns exist
if not all(col in data.columns for col in ["X", "Y_target", "Y_approx"]):
    print("Error: CSV file must contain columns 'X', 'Y_target', 'Y_approx'.")
    exit(1)

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(data["X"], data["Y_target"], label="Target: sin(x) + cos(2.5x)", color="blue", linewidth=2)
plt.plot(data["X"], data["Y_approx"], label="Approximated Function", color="red", linestyle="--", linewidth=2)

# Add labels and title
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Comparison of Target and Approximated Functions")
plt.legend()
plt.grid(True)

# Save the plot to a file
plt.savefig("function_comparison.png")
print("Plot saved as function_comparison.png")

# Display the plot
plt.show()
