"""Regenerate the published at-speed comparison from its recorded summary JSON."""
from pathlib import Path
import argparse,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--results',type=Path,default=Path(__file__).resolve().parents[1]/'results/at-speed-v2')
a=p.parse_args();out=a.results
data=json.loads((out/'summary.json').read_text())
labels=['Stationary · 10 m/s / 30 m','Stationary · 15 m/s / 45 m','Lower friction · 15 / 45','Short gap · 15 / 18','Severe stress · 15 / 9','Moving lead target · 15 / 30','Braking lead target · 15 / 30','Sudden target · 10 / 22','Range delayed by 256 ms','Range dropout · 200 ms','Noisy range · SD 0.5 m']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'axes.edgecolor':'#d8d0da','text.color':'#332b3b','savefig.facecolor':'#fcfaf6','figure.facecolor':'#fcfaf6'})
fig=plt.figure(figsize=(8,9.8));ax=fig.add_axes([.035,.13,.93,.72]);ax.set_xlim(0,1);ax.set_ylim(-.5,10.8);ax.axis('off')
fig.text(.035,.96,'66 at-speed Webots runs',fontsize=24,weight='bold')
fig.text(.035,.915,'11 designed conditions × 3 physics seeds × 2 policies',fontsize=13)
fig.text(.035,.88,'Condition (requested m/s / reference gap m)',fontsize=10)
fig.text(.51,.88,'Legacy TTC',fontsize=12,weight='bold',color='#396981',ha='center');fig.text(.76,.88,'Stopping envelope',fontsize=12,weight='bold',color='#b25d36',ha='center')
for i,label in enumerate(labels):
 y=10-i;ax.axhline(y-.48,color='#d8d0da',lw=.6);ax.text(0,y+.1,label,fontsize=11.8,va='center')
 for method,x in [('legacy_ttc',.51),('stopping_envelope',.78)]:
  row=data['policies'][method]['conditions'][i];passed=row['pass'];gap=row['minimum_hull_clearance_m'];color='#396981' if passed==3 else '#b44b34'
  ax.text(x,y+.17,f'{passed}/3 pass',fontsize=12.5,weight='bold',color=color,ha='center')
  ax.text(x,y-.12,f'Min gap {gap:.2f} m',fontsize=11.5,ha='center')
fig.text(.035,.107,'Each policy: 27/33 pass; 6 contacts. Both fail at 15/18 and 15/9.',fontsize=11.5,weight='bold',va='top')
fig.text(.035,.078,'Gap = signed collision-hull X clearance; negative values indicate overlap.\nMeasured cruise before scripted target appearance. Seeds repeat this designed family.\nMoving targets are kinematic boxes; software sensor faults and friction parameters.',fontsize=9.5,linespacing=1.5,va='top')
fig.savefig(out/'at-speed-results.png',dpi=180);plt.close(fig)
# Mobile comparison is a textual rendering of the same 22 recorded cells.
import html
s=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 400 1170" role="img" aria-labelledby="title desc"><title id="title">At-speed braking: every condition</title><desc id="desc">Each policy passes 27 of 33 executions and has six contacts. Short-gap 15 m/s cases fail for both policies. Rows show pass counts and worst signed collision-hull clearance across three physics seeds.</desc><rect width="400" height="1170" fill="#fcfaf6"/><g font-family="Arial,sans-serif" fill="#332b3b">']
def text(x,y,t,size=16,extra=''):return f'<text x="{x}" y="{y}" font-size="{size}" {extra}>{html.escape(t)}</text>'
s += [text(20,35,'At-speed braking',23,'font-weight="700"'),text(20,64,'66 runs / 11 conditions / 3 physics seeds',15)]
for i,label in enumerate(labels):
 y=105+i*82;s.append(text(20,y,label,16,'font-weight="700"'))
 for name,method,color,delta in [('TTC','legacy_ttc','#396981',24),('Envelope','stopping_envelope','#b25d36',46)]:
  row=data['policies'][method]['conditions'][i]
  s.append(text(20,y+delta,f"{name}: {row['pass']}/3 pass · min gap {row['minimum_hull_clearance_m']:.2f} m",15,f'fill="{color}"'))
 s.append(f'<line x1="20" y1="{y+62}" x2="378" y2="{y+62}" stroke="#d8d0da"/>')
s += [text(20,1040,'Each policy: 27/33 pass; 6 contacts.',16,'font-weight="700"'),text(20,1070,'Gap uses the collision hull, not vehicle reference.',14),text(20,1093,'Measured cruise, scripted kinematic targets.',14),text(20,1116,'Seeds repeat a small designed scenario family.',14),text(20,1139,'No road-safety or weather validation claim.',14),'</g></svg>']
(out/'at-speed-mobile.svg').write_text(''.join(s))
