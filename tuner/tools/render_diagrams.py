"""
tuner/tools/render_diagrams.py — render the docs' Mermaid charts to SVG.

VS Code's built-in Markdown preview does not draw Mermaid, so every chart in
docs/ is shown as an SVG from docs/diagrams/, with its Mermaid source kept
underneath in a collapsed <details> block. The source is the copy to edit;
this tool regenerates the image from it.

Run from the repo root:

    python3 -m tuner.tools.render_diagrams           # render new or changed charts
    python3 -m tuner.tools.render_diagrams --all     # re-render every chart
    python3 -m tuner.tools.render_diagrams --check   # exit 1 if any SVG is missing or stale

A bare ```mermaid block (a new chart) is wrapped in place: an image line and a
<details> block are inserted around it, and the image is named
<doc>_<nearest heading>.svg. After that the name in the image line is the
chart's identity, so headings can be renamed freely.

Staleness is tracked by docs/diagrams/manifest.json, which stores the SHA-1
of each chart's source at the time its SVG was rendered. Comparing hashes,
not timestamps, because a git checkout rewrites every mtime.

Needs Node.js (npx). Under WSL with the Windows npx on PATH, files are staged
through the Windows temp directory, because Windows node cannot use a
\\\\wsl$ working directory. Set MMDC to override the render command.
"""

import argparse
import hashlib
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOCS = os.path.join(ROOT, 'docs')
OUT_DIR = os.path.join(DOCS, 'diagrams')
MANIFEST = os.path.join(OUT_DIR, 'manifest.json')

MMDC_DEFAULT = 'npx -y -p @mermaid-js/mermaid-cli@11.4.2 mmdc'
# htmlLabels false (top level for mermaid 11, flowchart-level for older): the SVG then draws labels as SVG text, not embedded HTML,
# which some Markdown viewers strip.
MMDC_CONFIG = {'htmlLabels': False, 'flowchart': {'htmlLabels': False}}
SUMMARY = ('Mermaid source (edit this, then run '
           '<code>python3 -m tuner.tools.render_diagrams</code>)')

FENCE = re.compile(r'^```mermaid\n(.*?)^```\n', re.S | re.M)
IMAGE = re.compile(r'!\[[^\]]*\]\(([^)]*diagrams/([\w-]+)\.svg)\)\s*\n\s*<details>\s*\n'
                   r'<summary>[^\n]*</summary>\s*\n\s*$')
HEADING = re.compile(r'^#{1,6}\s+(.*)$', re.M)


def doc_files():
    """Every Markdown file under docs/ except the frozen logs."""
    for dirpath, _dirs, files in os.walk(DOCS):
        if os.path.relpath(dirpath, DOCS).split(os.sep)[0] == 'logs':
            continue
        for f in sorted(files):
            if f.endswith('.md'):
                yield os.path.join(dirpath, f)


def slugify(text):
    text = re.sub(r'`|\[|\]\([^)]*\)', '', text).lower()
    return re.sub(r'[^a-z0-9]+', '_', text).strip('_')[:40] or 'chart'


def sha1(src):
    return hashlib.sha1(src.encode('utf-8')).hexdigest()


def find_charts(path, taken):
    """
    Return (charts, new_text). charts is a list of (name, source). Bare blocks
    get an image line and <details> wrapper in new_text; wrapped ones keep
    the name from their image line.
    """
    text = open(path, encoding='utf-8').read()
    stem = os.path.splitext(os.path.basename(path))[0]
    rel_img_dir = os.path.relpath(OUT_DIR, os.path.dirname(path)).replace(os.sep, '/')
    charts, pieces, pos = [], [], 0
    for m in FENCE.finditer(text):
        src = m.group(1)
        before = text[:m.start()]
        wrapped = IMAGE.search(before[-600:])
        if wrapped:
            name = wrapped.group(2)
            pieces.append(text[pos:m.end()])
        else:
            headings = HEADING.findall(before)
            base = f'{stem}_{slugify(headings[-1]) if headings else "chart"}'
            name, n = base, 2
            while name in taken:
                name, n = f'{base}_{n}', n + 1
            pieces.append(text[pos:m.start()])
            pieces.append(
                f'![{headings[-1] if headings else stem}]({rel_img_dir}/{name}.svg)\n\n'
                f'<details>\n<summary>{SUMMARY}</summary>\n\n'
                f'{m.group(0)}\n</details>\n')
        taken.add(name)
        charts.append((name, src))
        pos = m.end()
    pieces.append(text[pos:])
    return charts, ''.join(pieces)


def mmdc_command():
    return shlex.split(os.environ.get('MMDC', MMDC_DEFAULT))


def staging_dir():
    """A temp dir the render command can use as its working directory."""
    npx = shutil.which(mmdc_command()[0]) or ''
    if npx.startswith('/mnt/') and shutil.which('wslpath'):
        win_tmp = subprocess.run(['cmd.exe', '/c', 'echo %TEMP%'], capture_output=True,
                                 text=True, cwd='/mnt/c').stdout.strip()
        base = subprocess.run(['wslpath', '-u', win_tmp], capture_output=True,
                              text=True).stdout.strip()
        return tempfile.mkdtemp(prefix='render_diagrams_', dir=base)
    return tempfile.mkdtemp(prefix='render_diagrams_')


def render(jobs):
    """Render [(name, source)] into docs/diagrams/<name>.svg. Returns failed names."""
    tmp = staging_dir()
    failed = []
    try:
        with open(os.path.join(tmp, 'config.json'), 'w') as f:
            json.dump(MMDC_CONFIG, f)
        for name, src in jobs:
            with open(os.path.join(tmp, f'{name}.mmd'), 'w', encoding='utf-8') as f:
                f.write(src)
            cmd = mmdc_command() + ['-q', '-b', 'white', '-c', 'config.json',
                                    '-i', f'{name}.mmd', '-o', f'{name}.svg']
            res = subprocess.run(cmd, cwd=tmp, capture_output=True, text=True)
            out = os.path.join(tmp, f'{name}.svg')
            if res.returncode != 0 or not os.path.isfile(out):
                failed.append(name)
                print(f'FAILED {name}:\n{res.stderr.strip()[-800:]}', file=sys.stderr)
                continue
            shutil.copyfile(out, os.path.join(OUT_DIR, f'{name}.svg'))
            print(f'rendered {name}.svg')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return failed


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--all', action='store_true', help='re-render every chart')
    ap.add_argument('--check', action='store_true',
                    help='report missing or stale SVGs and exit 1 if any, change nothing')
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    manifest = json.load(open(MANIFEST)) if os.path.isfile(MANIFEST) else {}
    taken, charts, rewrites = set(), [], {}
    for path in doc_files():
        found, new_text = find_charts(path, taken)
        rel = os.path.relpath(path, ROOT)
        charts += [(name, src, rel) for name, src in found]
        if new_text != open(path, encoding='utf-8').read():
            rewrites[path] = new_text

    stale = [(n, s, d) for n, s, d in charts
             if args.all or manifest.get(n, {}).get('sha1') != sha1(s)
             or not os.path.isfile(os.path.join(OUT_DIR, f'{n}.svg'))]
    names = {n for n, _s, _d in charts}
    orphans = sorted(f[:-4] for f in os.listdir(OUT_DIR)
                     if f.endswith('.svg') and f[:-4] not in names)

    if args.check:
        for n, _s, d in stale:
            print(f'stale or missing: {n}.svg ({d})')
        for path in rewrites:
            print(f'unwrapped chart in {os.path.relpath(path, ROOT)}')
        for n in orphans:
            print(f'orphan SVG (no chart uses it): {n}.svg')
        print(f'{len(charts)} charts, {len(stale)} stale, {len(rewrites)} unwrapped docs, '
              f'{len(orphans)} orphans')
        sys.exit(1 if stale or rewrites or orphans else 0)

    for path, text in rewrites.items():
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        print(f'wrapped new charts in {os.path.relpath(path, ROOT)}')

    failed = render([(n, s) for n, s, _d in stale])
    for n, s, d in charts:
        if n not in failed and any(n == x for x, _s, _d in stale):
            manifest[n] = {'doc': d, 'sha1': sha1(s)}
    for n in list(manifest):
        if n not in names:
            del manifest[n]
    with open(MANIFEST, 'w') as f:
        json.dump(dict(sorted(manifest.items())), f, indent=1)
        f.write('\n')
    for n in orphans:
        print(f'orphan SVG, delete it if the chart was removed: docs/diagrams/{n}.svg')
    print(f'{len(charts)} charts, {len(stale) - len(failed)} rendered, {len(failed)} failed')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
