"""Plot measured Webots stopping distances; requires matplotlib==3.11.2."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1] / 'results/webots-physics-v1'
cases = json.loads((ROOT / 'matrix-after.json').read_text())['cases']
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 13,
                     'axes.spines.top': False, 'axes.spines.right': False})
fig, (ax, values) = plt.subplots(1, 2, figsize=(4.8, 7.5),
                               gridspec_kw={'width_ratios': [4.3, 1.1]}, sharey=True)
fig.subplots_adjust(left=.21, right=.98, top=.69, bottom=.23, wspace=.10)
fig.text(.06, .97, 'Nine physics runs.\nNine controlled stops.', fontsize=18, weight='bold', va='top', linespacing=1.3)
fig.text(.06, .84, 'No obstacle contact. Starts from rest.\nStatic world, seed 1. Official fixed nightly.', fontsize=11, linespacing=1.5, va='top')
for index, case in enumerate(cases):
    distance = case['summary']['minimum_true_distance_m']
    color = {5.: '#146c94', 10.: '#745c9e', 15.: '#aa592b'}[case['speed_mps']]
    ax.barh(index, distance, height=.57, color=color)
    ax.text(distance+.22, index, f'{distance:.2f}', va='center', fontsize=12)
    values.text(.5, index, f"{case['summary']['maximum_speed_mps']:.2f}", ha='center', va='center', fontsize=12)
ax.set_yticks(range(9), [f"{c['speed_mps']:g} / {c['initial_reference_clearance_m']:g}" for c in cases])
ax.invert_yaxis()
ax.set_xlim(0, 20)
ax.set_xticks([0, 5, 10, 15, 20])
ax.set_xlabel('Minimum reference distance (m)', labelpad=12, fontsize=12)
ax.set_ylabel('Requested target (m/s) / initial gap (m)', labelpad=8, fontsize=12)
ax.axvline(3, color='#242c27', linestyle='--', linewidth=1)
ax.text(3.2, -.8, '3 m rule', fontsize=12)
ax.grid(axis='x', alpha=.16)
ax.set_axisbelow(True)
values.set_xlim(0, 1)
values.set_axis_off()
values.set_title('Actual peak\nspeed (m/s)', fontsize=11, pad=15)
fig.text(.06, .04, 'Reference = obstacle front face to vehicle reference X.\nNot bumper clearance. Targets are not all reached.', fontsize=10.5, linespacing=1.5, va='bottom')
fig.savefig(ROOT / 'stopping-distances.png', dpi=180, bbox_inches='tight', pad_inches=.14)
plt.close(fig)
