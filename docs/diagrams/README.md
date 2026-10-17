# Rendered diagrams

SVG copies of every Mermaid chart in `docs/`, so the docs show diagrams in any Markdown viewer. VS Code's built-in preview does not draw Mermaid. Each doc shows the SVG with the Mermaid source underneath in a collapsed "Mermaid source" block. **The source in the doc is the copy to edit.** The SVGs and `manifest.json` are generated.

## Changing or adding a chart

1. Edit the Mermaid block in the doc. For a new chart, write a plain ```` ```mermaid ```` block. The tool wraps it in the image line and collapsed block.
2. Run from the repo root (needs Node.js for `npx`):

```bash
python3 -m tuner.tools.render_diagrams           # render new or changed charts
python3 -m tuner.tools.render_diagrams --check   # exit 1 if any SVG is missing, stale or orphaned
python3 -m tuner.tools.render_diagrams --all     # re-render everything (after a settings change)
```

3. Commit the doc, the SVG and `manifest.json` together.

## How it works

- **Names.** A new chart's SVG is named `<doc>_<nearest heading>.svg`. After that, the name in the image line is the chart's identity, so a heading can be renamed without breaking it.
- **Staleness.** `manifest.json` stores the SHA-1 of each chart's source when its SVG was rendered. Hashes, not file times, because a git checkout rewrites every file time.
- **Renderer.** The Mermaid CLI (npm package @mermaid-js/mermaid-cli, version 11.4.2) through `npx`, white background, `htmlLabels: false` so labels are SVG text and not embedded HTML, which some viewers drop. Under WSL with the Windows `npx`, files are staged through the Windows temp directory. Set `MMDC` to use a different render command.
- **Removed charts.** The tool reports an SVG no chart uses any more as an orphan. Delete it by hand.

`docs/logs/` is a frozen record and is not scanned.
