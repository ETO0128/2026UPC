"""Plot flight-environment requirements from the saved full-flight records."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / "results/p3/safety"
summary = json.loads((DIRECTORY / "summary.json").read_text(encoding="utf-8"))


def load_trace(name):
    with (DIRECTORY / name).open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0] if key != "phase"}


p2 = load_trace("p2-nominal.csv")
p3 = load_trace("tracking-nominal.csv")
nominal = next(case for case in summary["cases"] if case["actualWindMps"] == 0)
ignition = nominal["ignitionTimeS"]
blue, teal, orange = "#3265a8", "#007f7a", "#c45d26"
plt.rcParams.update({"font.size": 13, "axes.spines.top": False, "axes.spines.right": False})
fig, axes = plt.subplots(2, 2, figsize=(10.2, 7.1))

for ax, key, scale, title, ylabel in [
    (axes[0, 0], "qPa", 1e3, "(a) Dynamic pressure", "Dynamic pressure (kPa)"),
    (axes[0, 1], "loadG", 1, "(b) Specific force at the centre of mass", "Non-gravitational specific force (g0)"),
    (axes[1, 1], "flowKineticFluxWm2", 1e6, "(d) Incoming flow energy", "Free-stream kinetic-energy flux (MW/m²)"),
]:
    ax.plot(p2["timeS"], p2[key] / scale, color=blue, linestyle="--", linewidth=1.4, label="P2 nominal, 26.23 t")
    ax.plot(p3["timeS"], p3[key] / scale, color=teal, linewidth=1.5, label="P3 feedback nominal, 30 t")
    peak = nominal["peaks"][key]
    ax.scatter([peak["at"]["timeS"]], [peak["value"] / scale], color=teal, s=22, zorder=5)
    ax.axvline(ignition, color="#777777", linewidth=.8, linestyle=":")
    ax.set(xlim=(0, 210), ylim=(0, None), title=title, ylabel=ylabel, xlabel="Time after separation (s)")
    ax.grid(alpha=.18)

axes[0, 0].legend(loc="upper left", fontsize=10, frameon=False)
axes[0, 0].text(.03, .65, "P3 ignition: 174.14 s\nDotted line in (a), (b), (d)", transform=axes[0, 0].transAxes, fontsize=10, color="#555555")
axes[0, 1].annotate("8.132 g0 at 181.58 s", xy=(181.577, 8.132), xytext=(45, 7.5),
                    fontsize=10, arrowprops={"arrowstyle": "-", "color": "#777777"})
axes[1, 1].text(.03, .87, "Flow indicator; not wall heat flux", transform=axes[1, 1].transAxes, fontsize=10, color=orange)

ax = axes[1, 0]
mask = (p3["timeS"] >= 166) & (p3["timeS"] <= 190)
ax.plot(p3["timeS"][mask], np.abs(p3["normalForceN"][mask]) / 1e6, color=teal, linewidth=1.5, label="Normal aerodynamic force")
ax.set(xlim=(166, 190), ylim=(0, 11), title="(c) Normal loading around the burn", ylabel="Normal aerodynamic force magnitude (MN)", xlabel="Time after separation (s)")
ax.axvline(ignition, color="#777777", linewidth=.8, linestyle=":")
ax.grid(alpha=.18)
right = ax.twinx()
right.plot(p3["timeS"][mask], p3["incidenceDeg"][mask], color=orange, linewidth=1.2, linestyle="--", label="Axis/flow acute angle")
right.set(ylim=(0, 40), ylabel="Axis/flow acute angle (deg)")
right.spines["right"].set_visible(True)
lines = ax.get_lines()[:1] + right.get_lines()
ax.legend(lines, [line.get_label() for line in lines], fontsize=9.5, loc="upper left", frameon=False)
fig.tight_layout(pad=1.2, h_pad=1.4, w_pad=1.2)
fig.savefig(ROOT / "figure/p3-flight-exposure.png", dpi=220)
plt.close(fig)
print("Generated figure/p3-flight-exposure.png")
