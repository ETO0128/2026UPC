"""Plot accepted effective-Isp scenario inventories and retained propellant."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
summary = json.loads((ROOT / "results/p3/propulsion/summary.json").read_text(encoding="utf-8"))
cases = summary["cases"]
if [case["ispS"] for case in cases] != summary["scenarioValues"] or not all(case["accepted"] for case in cases):
    raise RuntimeError("The figure requires accepted results for all three P0 Isp scenarios")
loads = np.array([case["propellantKg"] / 1000 for case in cases])
consumed = np.array([case["consumedKg"] / 1000 for case in cases])
retained = np.array([case["retainedUsableKg"] / 1000 for case in cases])
y = np.arange(len(cases))
plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(figsize=(7.3, 3.45))
ax.barh(y, consumed, height=.55, color="#c8733e", label="Consumed")
ax.barh(y, retained, left=consumed, height=.55, color="#14877f", label="Retained at contact")
for i, (load, burn, remaining) in enumerate(zip(loads, consumed, retained)):
    ax.text(burn / 2, i, f"{burn:.2f}", ha="center", va="center", color="white", fontsize=11)
    ax.text(burn + remaining / 2, i, f"{remaining:.2f}", ha="center", va="center", color="white", fontsize=10)
    ax.text(load + .3, i, f"{load:.2f} t", va="center", fontsize=11)
ax.set(yticks=y, yticklabels=[f"Isp = {case['ispS']} s" for case in cases],
       xlabel="Separation propellant, including retained inventory (t)", xlim=(0, max(loads) * 1.18))
ax.invert_yaxis()
ax.set_ylim(len(cases) - .5, -1.1)
ax.legend(loc="upper left", ncol=2, frameon=False, fontsize=10)
ax.set_axisbelow(True)
ax.grid(axis="x", alpha=.18)
fig.tight_layout()
fig.savefig(ROOT / "figure/p3-propulsion-budget.png", dpi=220)
plt.close(fig)
print("Generated figure/p3-propulsion-budget.png")
