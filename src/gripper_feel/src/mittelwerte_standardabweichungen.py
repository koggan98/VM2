import os
import pandas as pd
import numpy as np

# Pfad zum sensor_logs-Ordner
script_dir = os.path.dirname(os.path.abspath(__file__))
log_dir = os.path.join(script_dir, "sensor_logs","2N_threshold")

# Funktion, um Mittelwert & Standardabweichung für eine Kategorie zu berechnen
def analyze_category(category_prefix):
    files = [f for f in os.listdir(log_dir) if f.startswith(category_prefix) and f.endswith(".csv")]
    
    if not files:
        print(f"⚠️ Keine Dateien für {category_prefix} gefunden.")
        return None
    
    all_data = []
    
    for file in files:
        csv_path = os.path.join(log_dir, file)
        df = pd.read_csv(csv_path)
        all_data.append(df[["Force X", "Force Y", "Force Z", "Torque X", "Torque Y", "Torque Z"]])
    
    # Alle Datenpunkte zusammenfügen
    combined_df = pd.concat(all_data, ignore_index=True)
    
    # Mittelwert & Standardabweichung berechnen
    mean_values = combined_df.mean()
    std_values = combined_df.std()
    
    print(f"\n📊 Analyse für {category_prefix}:")
    print(mean_values)
    print("Standardabweichung:")
    print(std_values)
    
    return mean_values, std_values

# Kategorien analysieren
analyze_category("Hammer_links")
analyze_category("Hammer_rechts")
analyze_category("Schere_links")
analyze_category("Schere_rechts")
