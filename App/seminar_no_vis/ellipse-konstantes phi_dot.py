import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. Definition der Ellipsenparameter ---
A = 100.0  # Hauptachse
B = 50.0   # Nebenachse
T = 40.0  # Umlaufzeit X in Sekunden
dt = 0.05 # Zeitschritt

a = A / 2
b = B / 2

# --- 2. Berechnung von Position und (vereinfachter) Geschwindigkeit ---
# Wir ignorieren die Kepler-Gleichung und setzen phi (Exzentrische Anomalie)
# direkt proportional zur Zeit (Konstante Winkelgeschwindigkeit Omega).
omega = 2 * np.pi / T # Konstante Winkelgeschwindigkeit der Anomalie^

if 5 > 7:
    print("Five is greater than two!")
else:
    print("Bedingung nicht erfüllt")
print("Ende")

def calculate_state_simplified(t, a, b, omega):
    """
    Berechnet Position und Geschwindigkeit unter der Annahme,
    dass die exzentrische Anomalie phi linear mit der Zeit zunimmt.
    """
    # Exzentrische Anomalie phi (entspricht dem gewünschten linearen Winkelwachstum)
    phi = omega * t
    
    # Position (x, y) mit phi
    x = a * np.cos(phi)
    y = b * np.sin(phi)
    
    # Winkelrate d(phi)/dt ist konstant: omega
    dot_phi = omega 
    
    # Geschwindigkeitskomponenten (mit konstanter dot_phi = omega)
    # Beachte: Die Bahngeschwindigkeit |v| ist nicht konstant!
    vx = -a * np.sin(phi) * dot_phi
    vy = b * np.cos(phi) * dot_phi
    
    return x, y, vx, vy

# ----------------------------------------------------
# --- 3. Animations-Setup ---
# ----------------------------------------------------

fig, ax = plt.subplots(figsize=(8, 6))

# Ellipsenbahn als statischer Hintergrund
theta = np.linspace(0, 2 * np.pi, 100)
ellipse_x = a * np.cos(theta)
ellipse_y = b * np.sin(theta)
ax.plot(ellipse_x, ellipse_y, 'k--', alpha=0.5, label='Ellipsenbahn')

# Initialisierung des Punktes und des Geschwindigkeitsvektors
line, = ax.plot([], [], 'ro', markersize=8, label='Roboter') 
quiver = ax.quiver(0, 0, 0, 0, color='r', angles='xy', scale_units='xy', scale=0.5, width=0.005, headwidth=3, headlength=5)

ax.set_title(f'Ellipsenbewegung mit konstanter Winkelrate (phi) | T={T}s')
ax.set_xlabel('X-Position [cm]')
ax.set_ylabel('Y-Position [cm]')
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(-(a + 1), a + 1)
ax.set_ylim(-(b + 1), b + 1)
ax.legend(loc='lower right')
ax.grid(True, linestyle=':', alpha=0.6)

# Liste der Zeitpunkte für die Frames
num_frames = int(T / dt) * 2 
time_points = np.linspace(0, T * 2, num_frames)

# --- 4. Update-Funktion für jeden Frame ---
def update(frame_index):
    t = time_points[frame_index]
    
    # Zustand berechnen mit dem vereinfachten Modell
    x, y, vx, vy = calculate_state_simplified(t, a, b, omega)
    
    line.set_data([x], [y])
    
    scale_factor = 0.5 
    
    # Geschwindigkeitsvektor aktualisieren
    quiver.set_offsets(np.array([x, y]))
    quiver.set_UVC(vx * scale_factor, vy * scale_factor)
    
    v_betrag = np.sqrt(vx**2 + vy**2)
    ax.set_title(f'Vereinfachte Ellipsenbewegung | Zeit: {t:.2f} s | v-Betrag: {v_betrag:.2f} cm/s')
    
    return line, quiver

# --- 5. Erstellung der Animation ---
ani = FuncAnimation(fig, update, frames=len(time_points), interval=dt * 1000, blit=False)
plt.show()
# Da die Anzeige der Animation hier nicht funktioniert, zeigen wir nur den letzten Frame.

# Zeigen des letzten Frames zur Veranschaulichung
#t_end = time_points[-1]
#x_end, y_end, vx_end, vy_end = calculate_state_simplified(t_end, a, b, omega)
#ax.plot(x_end, y_end, 'go', markersize=8, label='Endpunkt')
#ax.quiver(x_end, y_end, vx_end * 0.5, vy_end * 0.5, color='g', angles='xy', scale_units='xy', scale=1, width=0.005, headwidth=6, headlength=10)

