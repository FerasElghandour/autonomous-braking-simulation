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
fig, (ax, values) = plt.subplots(1, 2, figsize=(6.2, 7.2),
                               gridspec_kw={'width_ratios': [4.3, 1.1]}, sharey=True)
fig.subplots_adjust(left=.18, right=.97, top=.73, bottom=.23, wspace=.08)
fig.text(.06, .97, 'Nine physics runs: nine controlled stops', fontsize=17, weight='bold', va='top')
fig.text(.06, .89, 'No obstacle contact. All runs start from rest.\nOne static world, seed 1; official fixed nightly build.', fontsize=11.5, linespacing=1.5, va='top')
for index, case in enumerate(cases):
    distance = case['summary']['minimum_true_distance_m']
    color = {5.: '#146c94', 10.: '#745c9e', 15.: '#aa592b'}[case['speed_mps']]
    ax.barh(index, distance, height=.57, color=color)
    ax.text(distance+.22, index, f'{distance:.2f}', va='center', fontsize=11.5)
    values.text(.5, index, f"{case['summary']['maximum_speed_mps']:.2f}", ha='center', va='center', fontsize=12)
ax.set_yticks(range(9), [f"{c['speed_mps']:g} / {c['initial_reference_clearance_m']:g}" for c in cases])
ax.invert_yaxis()
ax.set_xlim(0, 20)
ax.set_xticks([0, 5, 10, 15, 20])
ax.set_xlabel('Minimum reference distance (m)', labelpad=12, fontsize=12)
ax.set_ylabel('Requested target (m/s) / initial gap (m)', labelpad=10, fontsize=11)
ax.axvline(3, color='#242c27', linestyle='--', linewidth=1)
ax.text(3.2, -.8, '3 m acceptance rule', fontsize=10)
ax.grid(axis='x', alpha=.16)
ax.set_axisbelow(True)
values.set_xlim(0, 1)
values.set_axis_off()
values.set_title('Actual peak\nspeed (m/s)', fontsize=10.5, pad=15)
fig.text(.06, .04, 'Reference distance = obstacle front face to vehicle reference X.\nIt is not bumper clearance. Requested targets are not all reached.', fontsize=10.5, linespacing=1.5, va='bottom')
fig.savefig(ROOT / 'stopping-distances.png', dpi=180)
plt.close(fig)
