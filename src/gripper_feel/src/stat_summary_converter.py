# macht aus der summary stats csv ne schöne xlsx  
import os
import pandas as pd

# Pfad zum aktuellen Skript-Verzeichnis
script_dir = os.path.dirname(os.path.abspath(__file__))

# Pfad zur CSV-Datei mit den zusammengefassten Statistiken
csv_path = os.path.join(script_dir, "sensor_logs", "summary_stats.csv")

# CSV einlesen
df = pd.read_csv(csv_path)

# Spalten, die als Metainformation erhalten bleiben
meta_columns = ["Folder", "File"]

# Aufteilen in Gruppen anhand des Spaltennamens
max_columns = meta_columns + [col for col in df.columns if " max" in col]
min_columns = meta_columns + [col for col in df.columns if " min" in col]
mean_columns = meta_columns + [col for col in df.columns if " mean" in col]
std_columns = meta_columns + [col for col in df.columns if " std" in col]
var_columns = meta_columns + [col for col in df.columns if " var" in col]

# Ausgabe-Ordner für die Excel-Datei (wird im sensor_logs-Ordner abgelegt)
output_dir = os.path.join(script_dir, "sensor_logs")
os.makedirs(output_dir, exist_ok=True)  # Ordner erstellen, falls er nicht existiert

output_excel = os.path.join(output_dir, "summary_stats.xlsx")

with pd.ExcelWriter(output_excel) as writer:
    df[max_columns].to_excel(writer, sheet_name="Max", index=False)
    df[min_columns].to_excel(writer, sheet_name="Min", index=False)
    df[mean_columns].to_excel(writer, sheet_name="Mean", index=False)
    df[std_columns].to_excel(writer, sheet_name="Std", index=False)
    df[var_columns].to_excel(writer, sheet_name="Var", index=False)

print(f"Excel-Datei mit getrennten Kennzahlen wurde gespeichert unter: {output_excel}")
