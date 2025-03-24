import os
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

script_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.join(script_dir, "sensor_logs")

# Parameter
n_points = 50
columns = ["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"]

# Alle Unterordner durchgehen
for dirpath, dirnames, filenames in os.walk(root_dir):
    csv_files = [f for f in filenames if f.endswith(".csv")]
    
    if not csv_files:
        continue  # Skip Ordner ohne CSVs

    print(f"Bearbeite Ordner: {dirpath} ({len(csv_files)} Dateien)")
    
    interpolated_data = []

    for file in csv_files:
        path = os.path.join(dirpath, file)
        try:
            df = pd.read_csv(path)
            t = df["Time"].to_numpy()
            t_norm = (t - t[0]) / (t[-1] - t[0])
            t_target = np.linspace(0, 1, n_points)

            interpolated = {}
            for col in columns:
                y = df[col].to_numpy()
                f = interp1d(t_norm, y, kind='linear', fill_value="extrapolate")
                interpolated[col] = f(t_target)

            interpolated_data.append(interpolated)
        except Exception as e:
            print(f"⚠️ Fehler bei {file}: {e}")

    if not interpolated_data:
        continue

    # Mittelwert über alle Dateien
    mean_data = {col: np.mean([d[col] for d in interpolated_data], axis=0) for col in columns}
    mean_data["Time"] = np.linspace(0, 0.5, n_points)
    mean_df = pd.DataFrame(mean_data)

    # Output-Dateiname aus dem Ordnernamen generieren
    folder_name = os.path.basename(dirpath)
    mean_path = os.path.join(dirpath, f"mean_{folder_name}.csv")
    mean_df.to_csv(mean_path, index=False)
    print(f"Durchschnittsdatei gespeichert: {mean_path}")
