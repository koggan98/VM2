import os
import pandas as pd
import matplotlib.pyplot as plt

# Pfad zum sensor_logs-Ordner
script_dir = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(script_dir, "sensor_logs","daniel_3N")

# Hardcoded Dateiname
csv_filename = "pinzette_daniel_rechts10.csv"
csv_path = os.path.join(log_dir, csv_filename)

# Prüfen, ob Datei existiert
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"Fehler: Datei nicht gefunden! Gesuchter Pfad: {csv_path}")

# CSV-Datei laden
df = pd.read_csv(csv_path)

# Konvertiere Spalten in NumPy-Arrays
time_values = df["Time"].to_numpy()
force_x = df["Force X"].to_numpy()
force_y = df["Force Y"].to_numpy()
force_z = df["Force Z"].to_numpy()
torque_x = df["Torque X"].to_numpy()
torque_y = df["Torque Y"].to_numpy()
torque_z = df["Torque Z"].to_numpy()

# Plot erstellen
fig, ax1 = plt.subplots(figsize=(10, 5))

# Erste y-Achse für Kräfte
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Force (N)", color='tab:blue')
ax1.plot(time_values, force_x, label="Force X (N)", color='tab:red')
ax1.plot(time_values, force_y, label="Force Y (N)", color='tab:green')
ax1.plot(time_values, force_z, label="Force Z (N)", color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.legend(loc="upper left")
ax1.grid()

# Zweite y-Achse für Drehmomente
ax2 = ax1.twinx()
ax2.set_ylabel("Torque (Nm)", color='tab:red')
ax2.plot(time_values, torque_x, label="Torque X (Nm)", color='tab:red', linestyle="dashed")
ax2.plot(time_values, torque_y, label="Torque Y (Nm)", color='tab:green', linestyle="dashed")
ax2.plot(time_values, torque_z, label="Torque Z (Nm)", color='tab:blue', linestyle="dashed")
ax2.tick_params(axis='y', labelcolor='tab:red')
ax2.legend(loc="upper right")

# Titel setzen
plt.title(f"Force & Torque Sensor Data from {csv_filename}")

# Anzeigen
plt.show()
