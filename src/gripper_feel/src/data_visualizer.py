import os
import pandas as pd
import matplotlib.pyplot as plt

# Pfad des aktuellen Skripts
script_dir = os.path.dirname(os.path.abspath(__file__))

# Relativer Pfad zur CSV (hoch zum Hauptverzeichnis "CODE_VM2")
csv_path = os.path.abspath(os.path.join(script_dir, "../../../../force_data.csv"))

# Prüfen, ob Datei existiert
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"Fehler: Datei nicht gefunden! Gesuchter Pfad: {csv_path}")

# CSV-Datei laden
df = pd.read_csv(csv_path)

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
