import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Pfad zum sensor_logs-Ordner
script_dir = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(script_dir, "sensor_logs", "2N_threshold_daniel")

# Funktion, um alle relevanten Dateien zu laden und zu mitteln
def get_mean_force_torque(category_prefix):
    files = [f for f in os.listdir(log_dir) if f.startswith(category_prefix) and f.endswith(".csv")]
    
    if not files:
        print(f"⚠️ Keine Dateien für {category_prefix} gefunden.")
        return None
    
    all_data = []
    
    for file in files:
        csv_path = os.path.join(log_dir, file)
        df = pd.read_csv(csv_path)
        all_data.append(df[["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"]])
    
    # Alle Datenpunkte zusammenfügen und Mittelwerte berechnen
    combined_df = pd.concat(all_data, ignore_index=True)
    mean_values = combined_df.mean()
    
    return mean_values

# Mittelwerte berechnen
hammer_links_mean = get_mean_force_torque("Hammer_links")
schere_rechts_mean = get_mean_force_torque("Schere_rechts")

# Falls eine der Kategorien leer ist, abbrechen
if hammer_links_mean is None or schere_rechts_mean is None:
    print("⚠️ Eine oder mehrere Kategorien haben keine Daten. Abbruch.")
    exit()

# Balkendiagramm erstellen
labels = ["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"]
x = np.arange(len(labels))

fig, ax = plt.subplots(figsize=(10, 5))
bar_width = 0.4

ax.bar(x - bar_width/2, hammer_links_mean, width=bar_width, label="Hammer Links", color='blue')
ax.bar(x + bar_width/2, schere_rechts_mean, width=bar_width, label="Schere Rechts", color='orange')

ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.set_ylabel("Mittelwert (N oder Nm)")
ax.set_title("Vergleich: Hammer Links vs. Schere Rechts")
ax.legend()
ax.grid(axis="y", linestyle="--", alpha=0.7)

plt.show()
