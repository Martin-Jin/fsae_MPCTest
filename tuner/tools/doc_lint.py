"""
tuner/tools/doc_lint.py — flag docs and code that break the project's conventions.

Run from the repo root:

    python -m tuner.tools.doc_lint            # report only
    python -m tuner.tools.doc_lint --strict   # exit 1 if anything is flagged
    python -m tuner.tools.doc_lint --max 10   # stricter paragraph ceiling
    python -m tuner.tools.doc_lint docs/guides/tuning.md   # specific files

WHAT IT CHECKS
--------------
Docs (every README and docs/**/*.md except docs/logs/, which is a frozen
record and only its README is checked):

1. Long prose blocks: unstructured paragraphs longer than `--max` lines.
2. References to AI-assistant instruction files.
3. Transcript voice: first/second person and session scaffolding
   (docs/guides/getting_started.md may address the reader as "you").
4. Broken relative links and heading anchors.
5. Backticked file paths and `python -m` targets that do not exist.
6. Style: em dashes, more than one H1, intensifier words.
7. Module-reference coverage: every tracked .py/.sh/.launch.py/config file has
   an entry (its path in backticks) in docs/modules/*.md.

Code:

8. `from settings.<sub> import ...` anywhere: consumers must use
   `import settings; settings.X` so runtime overrides reach them.

WHY A SCRIPT
------------
These conventions drifted whenever they were only prose. A check that can be
run is enforceable in a way that a written rule is not.
"""
import argparse
import glob
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUTER_ROOT = os.path.dirname(ROOT)

LIST_OR_TABLE = re.compile(r'^\s*(\d+\.|[-*+]|\|)\s')
STRUCTURAL = ('#', '>', '```')
BANNED_REFS = ('CLAUDE.md', 'claude.md')
TRANSCRIPT = re.compile(
    r'\b(I |I\'ve |I\'d |we |we\'ve |our |you |you\'re |let\'s |as we )'
    r'|\bSession \d|\bthis session\b', re.I)
YOU_EXEMPT = ('docs/guides/getting_started.md',)
INTENSIFIERS = re.compile(r'\b(genuinely|actually|really|crucially)\b', re.I)
EM_DASH = '—'
LINK = re.compile(r'(?<!!)\[[^\]]*\]\(([^)\s]+)\)')
BACKTICK = re.compile(r'`([^`\n]+)`')
PYTHON_M = re.compile(r'python3?\s+-m\s+([A-Za-z_][\w.]*)')
PATH_EXT = ('.py', '.sh', '.md', '.yaml', '.yml', '.json', '.csv', '.txt', '.cfg',
            '.xml', '.launch.py', '.msg', '.Dockerfile')
SKIP_CHARS = re.compile(r'[*<>{}$~=:(),|\s\\]|\.\.\.|^https?|^-')
MODULE_REF_EXT = ('.py', '.sh', '.yaml', '.yml', '.cfg', '.xml', '.txt')
MODULE_REF_SKIP = ('__init__.py', '__main__.py')


def doc_files(explicit):
    if explicit:
        return explicit
    files = [f for f in ('README.md', 'fsds_simulator/README.md') if os.path.exists(f)]
    for f in sorted(glob.glob('docs/**/*.md', recursive=True)):
        if f.startswith('docs/logs/') and f != 'docs/logs/README.md':
            continue
        if f.startswith('docs/restructure'):
            continue
        files.append(f)
    return files


def tracked_files():
    out = subprocess.run(['git', 'ls-files'], capture_output=True, text=True, cwd=ROOT)
    return [f for f in out.stdout.split('\n') if f]


def prose_blocks(path):
    """Yield (line_no, n_lines) for each unstructured prose block in `path`."""
    lines = open(path, encoding='utf-8').read().split('\n')
    para, start, fenced = [], 0, False
    for i, line in enumerate(lines + ['']):
        if line.strip().startswith('```'):
            fenced = not fenced
            continue
        if fenced:
            continue
        if line.strip() == '':
            if para and not any(LIST_OR_TABLE.match(p) for p in para) \
                    and not any(p.lstrip().startswith(STRUCTURAL) for p in para):
                yield start + 1, len(para)
            para, start = [], i + 1
        else:
            if not para:
                start = i
            para.append(line)


def unfenced_lines(path):
    """Yield (line_no, text) for every line outside fenced code blocks."""
    fenced = False
    for i, line in enumerate(open(path, encoding='utf-8').read().split('\n'), 1):
        if line.strip().startswith('```'):
            fenced = not fenced
            continue
        if not fenced:
            yield i, line


def slug(heading):
    """GitHub-style heading anchor."""
    s = heading.strip().lower()
    s = re.sub(r'[`*_~]', '', s) if False else s.replace('`', '')
    s = re.sub(r'[^\w\- ]', '', s)
    return s.replace(' ', '-')


def anchors(path):
    seen, out = {}, set()
    for _, line in unfenced_lines(path):
        m = re.match(r'^#{1,6}\s+(.*?)\s*#*\s*$', line)
        if m:
            s = slug(m.group(1))
            n = seen.get(s, 0)
            seen[s] = n + 1
            out.add(s if n == 0 else f'{s}-{n}')
    return out


def check_links(path):
    """Yield (line_no, message) for each broken relative link or anchor."""
    base = os.path.dirname(path)
    for i, line in unfenced_lines(path):
        for target in LINK.findall(line):
            if re.match(r'^(https?:|mailto:)', target):
                continue
            file_part, _, anchor = target.partition('#')
            dest = path if file_part == '' else os.path.normpath(os.path.join(base, file_part))
            if not os.path.exists(dest):
                yield i, f'broken link: {target}'
                continue
            if anchor and dest.endswith('.md') and anchor not in anchors(dest):
                yield i, f'broken anchor: {target}'


def resolves(token, doc_path, basenames):
    """True if `token` names a file/dir that exists in any plausible root."""
    bases = [ROOT, os.path.join(ROOT, 'fsds_simulator'), os.path.join(ROOT, 'docs'),
             os.path.dirname(os.path.abspath(doc_path)), OUTER_ROOT,
             os.path.join(OUTER_ROOT, 'ros2'),
             os.path.join(OUTER_ROOT, 'ros2', 'src', 'fsae_planning'),
             os.path.join(ROOT, 'fsds_simulator', 'control', 'fsae_control'),
             os.path.join(ROOT, 'fsds_simulator', 'control', 'fsae_control', 'fsae_control')]
    for b in bases:
        if os.path.exists(os.path.join(b, token.rstrip('/'))):
            return True
    return '/' not in token and token in basenames


def check_paths(path, basenames):
    """Yield (line_no, message) for backticked paths and python -m targets that do not exist."""
    for i, line in unfenced_lines(path):
        for tok in BACKTICK.findall(line):
            tok = tok.strip()
            m = PYTHON_M.search(tok)
            if m:
                mod = m.group(1).replace('.', '/')
                if not (os.path.exists(os.path.join(ROOT, mod + '.py'))
                        or os.path.exists(os.path.join(ROOT, mod, '__main__.py'))):
                    yield i, f'python -m target not found: {m.group(1)}'
                continue
            if SKIP_CHARS.search(tok) or tok.startswith(('/', 'fsae_logs', '.')):
                continue
            if '/' not in tok and not tok.endswith(PATH_EXT):
                continue
            if not resolves(tok, path, basenames):
                yield i, f'path not found: {tok}'
        for m in PYTHON_M.finditer(line):
            if '`' in line:
                continue
            mod = m.group(1).replace('.', '/')
            if not (os.path.exists(os.path.join(ROOT, mod + '.py'))
                    or os.path.exists(os.path.join(ROOT, mod, '__main__.py'))):
                yield i, f'python -m target not found: {m.group(1)}'
    fenced_cmd = False
    for i, line in enumerate(open(path, encoding='utf-8').read().split('\n'), 1):
        if line.strip().startswith('```'):
            fenced_cmd = not fenced_cmd
            continue
        if fenced_cmd:
            for m in PYTHON_M.finditer(line):
                mod = m.group(1).replace('.', '/')
                if not (os.path.exists(os.path.join(ROOT, mod + '.py'))
                        or os.path.exists(os.path.join(ROOT, mod, '__main__.py'))):
                    yield i, f'python -m target not found: {m.group(1)}'


def check_style(path):
    """Yield (line_no, message) for em dashes, extra H1s and intensifiers."""
    h1 = 0
    for i, line in unfenced_lines(path):
        if re.match(r'^# ', line):
            h1 += 1
            if h1 > 1:
                yield i, 'more than one H1'
        if EM_DASH in line:
            yield i, 'em dash'
        m = INTENSIFIERS.search(re.sub(r'`[^`]*`', '', line))
        if m:
            yield i, f'intensifier: {m.group(0)}'


def module_ref_coverage():
    """Yield tracked source/config files that no docs/modules/*.md entry names."""
    mods = ''
    for f in glob.glob(os.path.join(ROOT, 'docs', 'modules', '*.md')):
        mods += open(f, encoding='utf-8').read()
    for f in tracked_files():
        if not f.endswith(MODULE_REF_EXT) and not f.endswith('.launch.py'):
            continue
        if os.path.basename(f) in MODULE_REF_SKIP:
            continue
        if f.startswith(('docs/', 'tracks/', 'settings_profiles/', 'fsds_simulator/tracks/',
                         'fsds_simulator/cone_maps/', 'fsds_simulator/recorded_runs/')):
            continue
        if f.endswith('.txt') and not f.endswith('requirements.txt'):
            continue
        if f.endswith('.bak'):
            continue
        if f'`{f}`' not in mods and f'`{os.path.basename(f)}`' not in mods:
            yield f


def check_code_imports():
    """Yield (file, line_no) for every `from settings.<sub> import`."""
    pat = re.compile(r'^\s*from\s+settings\.\w+\s+import')
    for f in tracked_files():
        if not f.endswith('.py') or f == 'tuner/tools/doc_lint.py':
            continue
        try:
            text = open(os.path.join(ROOT, f), encoding='utf-8').read()
        except OSError:
            continue
        if f.startswith('settings/'):
            continue
        for i, line in enumerate(text.split('\n'), 1):
            if pat.match(line):
                yield f, i


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--max', type=int, default=12,
                    help='longest allowed prose block, in lines (default: %(default)s)')
    ap.add_argument('--strict', action='store_true',
                    help='exit non-zero if anything is flagged')
    ap.add_argument('paths', nargs='*', default=None,
                    help='doc files to check (default: READMEs and docs/ except docs/logs/)')
    args = ap.parse_args()
    os.chdir(ROOT)

    files = doc_files(args.paths)
    basenames = {os.path.basename(f) for f in tracked_files()}
    live_tracks = os.path.join(OUTER_ROOT, 'ros2', 'src', 'fsae_planning', 'tracks')
    for _, _, names in os.walk(live_tracks):
        basenames.update(names)
    counts = {}

    def flag(kind, where, msg):
        counts[kind] = counts.get(kind, 0) + 1
        print(f'{where}: {msg}')

    for path in files:
        for line_no, n in prose_blocks(path):
            if n > args.max:
                flag('long-prose', f'{path}:{line_no}', f'prose block of {n} lines (max {args.max})')
        for i, line in unfenced_lines(path):
            if any(r in line for r in BANNED_REFS):
                flag('assistant-ref', f'{path}:{i}', 'references an AI-assistant instruction file')
            if path not in YOU_EXEMPT and TRANSCRIPT.search(re.sub(r'`[^`]*`', '', line)):
                flag('voice', f'{path}:{i}', 'first/second person or session scaffolding')
        for line_no, msg in check_links(path):
            flag('link', f'{path}:{line_no}', msg)
        for line_no, msg in check_paths(path, basenames):
            flag('path', f'{path}:{line_no}', msg)
        for line_no, msg in check_style(path):
            flag('style', f'{path}:{line_no}', msg)

    if not args.paths:
        for f in module_ref_coverage():
            flag('module-ref', f, 'no entry in docs/modules/*.md')
        for f, line_no in check_code_imports():
            flag('code-import', f'{f}:{line_no}', 'use `import settings; settings.X`, not `from settings.<sub> import`')

    total = sum(counts.values())
    print(f'\n{len(files)} docs checked, {total} issue(s): '
          + (', '.join(f'{k}={v}' for k, v in sorted(counts.items())) or 'none'))
    if args.strict and total:
        sys.exit(1)


if __name__ == '__main__':
    main()
