"""
tuner/tools/doc_lint.py — flag docs that break the project's writing conventions.

Run from the repo root:

    python -m tuner.tools.doc_lint            # report only
    python -m tuner.tools.doc_lint --max 10   # stricter paragraph ceiling

WHAT IT CHECKS
--------------
1. Prose blocks longer than `--max` lines (default 12). A wall of text does not
   get read; the convention is to break it into bullets, a table, or several
   short paragraphs. Lists, tables, headings, block quotes and fenced code are
   exempt -- those are already structured.
2. References to AI-assistant instruction files, which are not project
   documents and should not be cited as though they were.
3. Transcript voice -- first/second person and session scaffolding. These read
   as chat output rather than as a document someone wrote.

WHY A SCRIPT
------------
These conventions were previously stated in prose and drifted anyway: at the
time this was added, 196 prose blocks over 10 lines existed across docs/, most
of them predating the convention. A check that can be run is enforceable in a
way that a written rule is not.

Existing violations are NOT failures by default -- the exit code is 0 unless
`--strict` is passed -- so this can be adopted without a large mechanical
rewrite of historical investigation logs, whose measurement records are worth
more than their formatting.
"""
import argparse
import glob
import os
import re
import sys

LIST_OR_TABLE = re.compile(r'^\s*(\d+\.|[-*+]|\|)\s')
STRUCTURAL = ('#', '>', '```')
BANNED_REFS = ('CLAUDE.md', 'claude.md')
TRANSCRIPT = re.compile(
    r'\b(I |I\'ve |I\'d |we |we\'ve |our |you |you\'re |let\'s |as we )'
    r'|\bSession \d|\bthis session\b', re.I)


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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--max', type=int, default=12,
                    help='longest allowed prose block, in lines (default: %(default)s)')
    ap.add_argument('--strict', action='store_true',
                    help='exit non-zero if anything is flagged')
    ap.add_argument('paths', nargs='*', default=None,
                    help='files to check (default: every .md under docs/)')
    args = ap.parse_args()

    files = args.paths or sorted(glob.glob('docs/**/*.md', recursive=True))
    n_long = n_ref = n_voice = 0

    for f in files:
        if not os.path.isfile(f):
            continue
        text = open(f, encoding='utf-8').read()

        long_blocks = [(ln, n) for ln, n in prose_blocks(f) if n > args.max]
        refs = [i + 1 for i, l in enumerate(text.split('\n'))
                if any(b in l for b in BANNED_REFS)]
        voice = [i + 1 for i, l in enumerate(text.split('\n'))
                 if TRANSCRIPT.search(l) and not l.lstrip().startswith('>')]

        if long_blocks or refs or voice:
            print(f'{f}')
            for ln, n in long_blocks:
                print(f'    line {ln}: prose block of {n} lines (max {args.max})')
            for ln in refs:
                print(f'    line {ln}: reference to an AI-assistant instruction file')
            for ln in voice[:5]:
                print(f'    line {ln}: transcript voice (first/second person or session ref)')
            if len(voice) > 5:
                print(f'    ... and {len(voice) - 5} more transcript-voice lines')
        n_long += len(long_blocks)
        n_ref += len(refs)
        n_voice += len(voice)

    print(f'\n{len(files)} files checked: {n_long} oversized prose blocks, '
          f'{n_ref} assistant-file references, {n_voice} transcript-voice lines')
    if args.strict and (n_long or n_ref or n_voice):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
