import os
import pandas as pd
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.join(script_dir, "sensor_logs")

# Spalten, für die wir die Statistiken berechnen wollen
columns = ["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"]

# Hier sammeln wir alle Statistik-Daten, um sie später als DataFrame zu speichern
summary_data = []

for dirpath, dirnames, filenames in os.walk(root_dir):
    # Falls im Pfad "2N" vorkommt, überspringen wir diesen Ordner (z. B. 2N_threshold_daniel)
    if "2N" in dirpath:
        continue

    for filename in filenames:
        # Wir interessieren uns nur für Dateien, die mit 'mean_' beginnen und '.csv' enden
        if filename.startswith("mean_") and filename.endswith(".csv"):
            csv_path = os.path.join(dirpath, filename)

            try:
                df = pd.read_csv(csv_path)

                # Dictionary, das die Statistiken für diese Datei speichert
                stats = {}
                # Ordnername (z. B. 'hammer_links') als Kontext
                stats["Folder"] = os.path.basename(dirpath)
                # Dateiname (z. B. 'mean_hammer_links.csv') als Kontext
                stats["File"] = filename

                # Für jede gewünschte Spalte berechnen wir Max, Min, Mean, Std, Var
                for col in columns:
                    if col in df.columns:
                        stats[f"{col} max"] = df[col].max()
                        stats[f"{col} min"] = df[col].min()
                        stats[f"{col} mean"] = df[col].mean()
                        stats[f"{col} std"] = df[col].std()
                        stats[f"{col} var"] = df[col].var()
                    else:
                        # Falls die Spalte nicht vorhanden ist, kann man hier ggf. 0 oder NaN setzen
                        stats[f"{col} max"] = np.nan
                        stats[f"{col} min"] = np.nan
                        stats[f"{col} mean"] = np.nan
                        stats[f"{col} std"] = np.nan
                        stats[f"{col} var"] = np.nan

                # Das Dictionary fügen wir unserer Zusammenfassungsliste hinzu
                summary_data.append(stats)

            except Exception as e:
                print(f"Fehler beim Einlesen von {csv_path}: {e}")

# Aus den gesammelten Daten ein DataFrame erstellen
summary_df = pd.DataFrame(summary_data)

# Dieses DataFrame als CSV speichern
output_path = os.path.join(root_dir, "summary_stats.csv")
summary_df.to_csv(output_path, index=False)
print(f"Zusammenfassende Statistik-Datei gespeichert unter: {output_path}")
