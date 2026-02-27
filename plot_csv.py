import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

def plot_simulation(filename):
    # Pfad-Handling
    filepath = os.path.join("data", filename)
    
    if not os.path.exists(filepath):
        print(f"Error: File '{filepath}' not found.")
        return

    # Daten laden
    df = pd.read_csv(filepath)
    if len(df.columns) < 2:
        print("Error: CSV must have at least two columns (Time, Velocity)")
        return

    x_col, y_col = df.columns[0], df.columns[1]

    # --- Professionelles Styling ---
    plt.style.use('seaborn-v0_8-muted') # Moderner, cleaner Look
    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot der Daten
    ax.plot(df[x_col], df[y_col], label='Actual Velocity', 
            color='#007acc', linewidth=2, linestyle='-')

    # Optional: Zielgeschwindigkeit einzeichnen (falls bekannt)
    # Wenn dein CSV Header 'Target' hätte, könnte man das hier plotten.
    # Hier zeichnen wir eine gestrichelte Linie für den letzten Wert als "Steady State"
    plt.axhline(y=df[y_col].iloc[-1], color='r', linestyle='--', alpha=0.5, label='Goal Velocity')

    # Beschriftungen (Wichtig für Ingenieure: Einheiten!)
    ax.set_xlabel('Time [s]', fontsize=12, fontweight='bold')
    ax.set_ylabel('Velocity [m/s]', fontsize=12, fontweight='bold')
    ax.set_title(f'Cruise Control Step Response', fontsize=14, pad=15)
    
    # Gitter verfeinern
    ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)
    ax.legend(loc='lower right', frameon=True, shadow=True)

    # Layout optimieren
    plt.tight_layout()

    # --- Automatisches Speichern für GitHub ---
    output_name = filename.replace('.csv', '.png')
    output_path = os.path.join("data", output_name)
    plt.savefig(output_path, dpi=300) # Hohe Auflösung für Dokumentationen
    print(f"✅ Plot saved as: {output_path}")

    # Show (wird in WSL oft ignoriert, daher savefig als Backup)
    plt.show()

if __name__ == "__main__":
    # Argument Parser für professionelle CLI Bedienung
    parser = argparse.ArgumentParser(description='Plot Cruise Control Simulation Data')
    parser.add_argument('filename', type=str, help='Name of the CSV file in the data/ folder')
    
    args = parser.parse_args()
    plot_simulation(args.filename)