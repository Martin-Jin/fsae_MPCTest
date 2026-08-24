# Planning/Control Sync — moved

This document has been split by subject into [`docs/reference/`](reference/).

| you were looking for | now in |
|---|---|
| file mapping, non-mirrors, numeric-parity tables, score parity, resync procedure | [`reference/offline_live_parity.md`](reference/offline_live_parity.md) |
| speed profile, raceline/centreline exporters, `SPEED_CSV`/`PATH_CSV` | [`reference/reference_path_and_speed.md`](reference/reference_path_and_speed.md) |
| what a control mechanism does and why | [`reference/control_mechanisms.md`](reference/control_mechanisms.md) |
| sim-vs-car divergences, the lateral-acceleration ceiling, the planner defect | [`reference/simulator_fidelity.md`](reference/simulator_fidelity.md) |
| removed, superseded or rejected mechanisms | [`reference/superseded_mechanisms.md`](reference/superseded_mechanisms.md) |

See [`reference/README.md`](reference/README.md) for the index and for where
new content belongs.

**Why it was split.** The file stated its purpose as a resync reference but had
grown to 38 sections and 2214 lines, of which five were about resyncing. It had
become the default home for anything touching planning or control, which made
it both hard to navigate and easy to add unrelated material to.
