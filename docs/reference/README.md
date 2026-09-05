# Reference documents

Deep reference for the planning/control stack, split by subject. These replace
the former single `docs/planning_control_sync.md`, which had grown to 38
sections covering five unrelated topics.

| document | covers |
|---|---|
| [offline_live_parity.md](offline_live_parity.md) | What must stay matched between this repo and the live `fsae_planning` nodes: the `fsds_simulator/` mirror, the numeric-parity tables, score parity, and the resync procedure. |
| [reference_path_and_speed.md](reference_path_and_speed.md) | Where the car drives and how fast: the three-pass speed profile, the raceline/centreline exporters, and how to switch either. |
| [control_mechanisms.md](control_mechanisms.md) | Per-mechanism reference for what exists in the control stack today and why each is shaped as it is. |
| [simulator_fidelity.md](simulator_fidelity.md) | Where the offline simulator and the live car diverge, and which gaps are explained. |
| [superseded_mechanisms.md](superseded_mechanisms.md) | Mechanisms built and then removed, superseded or rejected. |

## Where new content belongs

| content | document |
|---|---|
| a weight, gain or flag and how to tune it | `docs/tuning.md` |
| how a subsystem is built | `docs/architecture.md` |
| how to run or export something | `docs/developer_guide.md` |
| an offline/live matching obligation | `offline_live_parity.md` |
| what a live mechanism does | `control_mechanisms.md` |
| a sim-vs-car discrepancy | `simulator_fidelity.md` |
| a mechanism that no longer exists | `superseded_mechanisms.md` |
| an investigation's history and measurements | `docs/logs/` |

Describe a mechanism in its own document and record only its parity obligation
in `offline_live_parity.md`. Writing both in one place is how the previous
single file accumulated 33 sections that had nothing to do with parity.
