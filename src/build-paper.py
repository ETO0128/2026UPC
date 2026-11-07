"""Compile paper/main.tex with the portable TinyTeX in .tools (two passes) and report problems."""
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TEX = ROOT / '.tools/TinyTeX/bin/windows/pdflatex.exe'
OUT = ROOT / 'paper/build/p5'
OUT.mkdir(parents=True, exist_ok=True)
env = dict(os.environ, TEXINPUTS=str(ROOT / '.tools/fhdr') + '//;')
for n in (1, 2):
    r = subprocess.run([str(TEX), '-interaction=nonstopmode', '-halt-on-error', f'-output-directory={OUT}',
                        'paper/main.tex'], cwd=ROOT, env=env, capture_output=True, text=True, errors='replace')
    if r.returncode:
        print(f'pass {n} failed'); print(r.stdout[-1500:]); sys.exit(1)
log = (OUT / 'main.log').read_text(errors='replace')
print('pages:', re.search(r'Output written on .*?\((\d+) pages', log).group(1))
for pat in (r'LaTeX Warning: Reference .*', r'LaTeX Warning: Citation .*', r'Overfull \\hbox .*', r'LaTeX Warning: There were undefined.*', r'Missing character.*'):
    for m in sorted(set(re.findall(pat, log)))[:12]:
        print(m)
