import pandas as pd
import matplotlib.pyplot as plt

# CSV-Datei laden
df = pd.read_csv("force_data.csv")

# Plot der Kraftwerte über die Zeit
plt.figure(figsize=(10, 5))
plt.plot(df["Time"], df["Force X"], label="Force X (N)")
plt.plot(df["Time"], df["Force Y"], label="Force Y (N)")
plt.plot(df["Time"], df["Force Z"], label="Force Z (N)")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Force Sensor Data over Time")
plt.legend()
plt.grid()
plt.show()
