# Audit: planning_control_sync.md, fsae_planning_pending_pr.md, steering_turn_in_upgrade_options.md

Repo root for `fsae_MPCTest` paths below: `/home/Formula-Student-Driverless-Simulator/fsae_MPCTest`.
Live tree: `/home/Formula-Student-Driverless-Simulator/ros2/src/fsae_planning`.

---

## Doc 1: `docs/planning_control_sync.md` (redirect stub)

**Purpose/audience:** pure redirect stub pointing to the split `docs/reference/` files. Audience: anyone who still has the old filename bookmarked.

**Check: do the 6 linked files exist?**

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 5 | `reference/offline_live_parity.md` exists | OK | `ls docs/reference/` shows `offline_live_parity.md` | none |
| 6 | `reference/reference_path_and_speed.md` exists | OK | present in listing | none |
| 7 | `reference/control_mechanisms.md` exists | OK | present in listing | none |
| 8 | `reference/simulator_fidelity.md` exists | OK | present in listing | none |
| 9 | `reference/superseded_mechanisms.md` exists | OK | present in listing | none |
| 13 | `reference/README.md` exists | OK | present in listing | none |

All 6 targets resolve. Doc 1 itself is accurate and small; nothing to fix in it directly. One extra file exists in `docs/reference/` that this stub's table omits: `simulator_glossary.md` (present on disk and listed in `reference/README.md`'s own table, just not in this stub's shorter table) — not wrong, just incomplete versus `reference/README.md`, not worth flagging as an error since the stub says "See reference/README.md for the index," i.e. it explicitly defers completeness to that file.

**Overlap:** none, this is a pure pointer.

**Readability:** fine, it's 15 lines.

**Mechanism-without-why:** N/A, no mechanism content.

**Code paths mentioned:** none (only doc paths).

### Cross-doc anchor rot check (high-value finding, spans repos beyond the 3 assigned docs)

`docs/reference/README.md`'s actual headings are almost entirely a table (piped list), not `##`/`###` sections — the only real heading besides the title is:

```
## Where new content belongs
```

There is **no** `## MPC weight/gain parity` heading, no `## Live/offline score parity`, no `## Nonlinear MPC (use_nmpc)`, no `## Three MPCC-inspired additions`, no `## Which settings affect which controller`, no `## Corner-factor...` heading, no `## slew-rate` heading, and no `## Known planner defect` heading in this file at all — that content was moved out to the five subject files (`offline_live_parity.md`, `control_mechanisms.md`, `simulator_fidelity.md`, etc.), and `README.md` is now just an index table with links to those files' own headings, not a document that carries the content itself.

**Every doc anchoring directly into `docs/reference/README.md#<section>` is broken**, because that content no longer lives in `README.md`, it lives in the linked subject files, each under its own heading. This is systemic dangling-anchor rot from the `planning_control_sync.md` → `reference/` split (commit `59dbd91` per git log) not being followed by an anchor-fixup pass elsewhere.

Broken/likely-broken anchor references found, by file:

| referencing doc | line | broken reference | what it should probably point at instead |
|---|---|---|---|
| `docs/fsae_planning_pending_pr.md` | 42 | `` docs/reference/README.md``'s "MPC weight/gain parity: MPCParams ↔ settings.py" table `` | `docs/reference/offline_live_parity.md` (that's where the parity table now lives per `reference/README.md`'s own index) |
| `docs/steering_turn_in_upgrade_options.md` | 174 | `` docs/reference/README.md``'s slew-rate section `` | likely `docs/reference/control_mechanisms.md` or `offline_live_parity.md`; not verified which subject file currently documents the slew-rate measurement, but definitely not `README.md` |
| `docs/tuning.md` | 171 | `` docs/reference/README.md``'s "Nonlinear MPC (use_nmpc)" section `` | `docs/nmpc.md` (README.md's own table maps NMPC detail there) |
| `docs/tuning.md` | 182 | `` docs/reference/README.md``'s "Three MPCC-inspired additions" subsection, and "Which settings affect which controller" `` | `docs/nmpc.md` for the MPCC content (per its own table row); "which settings affect which controller" content not located in this audit, needs a dedicated search |
| `docs/architecture.md` | 386 | `LapProgressTracker` in `` docs/reference/README.md``'s "Live/offline score parity" section `` | `docs/reference/offline_live_parity.md` |
| `docs/nmpc.md` | 254 | `` docs/reference/README.md``'s "Which settings affect which controller" map `` | not located; likely `control_mechanisms.md`, unverified |
| `docs/nmpc.md` | 270 | `` docs/reference/README.md``'s writeup [on MPCC rejection] `` | likely `docs/reference/superseded_mechanisms.md` (MPCC progress term was "DEFERRED", per user' memory note — also a possible **content** mismatch, see below) |
| `docs/logs/sim_to_real_investigation.md` | 956, 1348, 1401, 1453, 1488, 2028, 2179, 2568, 2867, 4280, 4766, 4824, 4863 | many `` docs/reference/README.md``'s "Known planner..." / exit-heading-boost / etc. `` and bare `` `docs/reference/` `` refs | these are historical-log entries (append-only investigation log); lower priority to fix since a log is a point-in-time record, but still technically dangling if a reader clicks through today |
| `docs/logs/late_turn_in_investigation.md` | 1143, 2610, 2834, 3213, 3249 | similar `docs/reference/README.md#`-style prose references | same category as above, log entries |

**Additional invalid Markdown link syntax** (bare directory link using backticks instead of a path, `[text](\`path\`)`, which renders as a literal-backtick link target, not a working relative link):

| file | line | text |
|---|---|---|
| `docs/architecture.md` | 26 | `` see [`docs/reference/`](docs/reference/) `` — actually this one uses a plain path, not backticks, so it likely resolves as a directory link (GitHub renders directory listings) — **not broken**, re-checked below |
| `docs/architecture.md` | 421 | `` See [`docs/reference/`](\`docs/reference/\`) `` — literal backticks INSIDE the parens, invalid target | fix to `(reference/)` or `(docs/reference/)` without backticks |
| `docs/logs/sim_to_real_investigation.md` | 14 | `` [`docs/reference/`](\`docs/reference/\`) `` — same backtick-in-target bug | same fix |
| `docs/logs/sim_to_real_investigation.md` | 4863 | `` [`docs/reference/`](\`docs/reference/\`) `` — same bug | same fix |

Several other docs use `[`docs/reference/`](reference/)` or `[`docs/reference/`](../reference/)` (no backticks inside parens) — those are syntactically valid relative directory links (`offline_guide.md:142`, `lmpc.md:475`, `fsds_integration_guide.md:29,331`) and **do** resolve to a real directory, so they're fine, just link to a directory rather than a specific anchor (imprecise, not broken).

**Net for doc 1's assigned check:** doc 1 itself (the stub) is 100% correct. The high-value finding is that `docs/reference/README.md` is now purely an index (no content headings), so **every other doc in the repo that anchors past `docs/reference/README.md#...`** into supposed content sections is dangling. This is systemic, not a one-off, and traces to the `59dbd91` split commit never being followed by a repo-wide anchor fixup.

**Proposed action for doc 1:** keep as-is (it's accurate). Separately (flagging for the orchestrator/other doc's owner since it's out of this doc's literal scope but was requested as a cross-check): do a repo-wide grep-and-fix pass replacing `docs/reference/README.md#<anchor>` with the correct subject file, using `reference/README.md`'s own index table as the mapping key.

---

## Doc 2: `docs/fsae_planning_pending_pr.md`

**Purpose/audience:** a summary of uncommitted/pending work in the live `fsae_planning` checkout, meant for someone tracking what still needs a PR into that repo. Audience: whoever eventually files that PR (probably the user, since agents can't push there).

**Overlap with other docs:** overlaps with `ros2/src/fsae_planning/CHANGES.md` itself (this doc is explicitly a summary of it) and with `docs/logs/late_turn_in_investigation.md` (NMPC introduction narrative) and `docs/reference/offline_live_parity.md` (parity claims).

### Major finding: the doc's premises are stale

The doc's header (line 3) claims:
1. Source is **uncommitted working-tree changes**, as of **2026-08-13**.
2. "Local `main` is up to date with `origin/main` (no upstream drift)."

Both are now false in ways that matter:

- `fsae_planning`'s current branch is **`feature/nmpc-and-controller-improvements`**, not `main`. (`git -C ros2/src/fsae_planning status` → `On branch feature/nmpc-and-controller-improvements`.)
- The content described as "pending/uncommitted" in this doc (the NMPC controller, `mpc_params.py`, delay compensation, scoring.py, etc.) has **since been committed**: `git log --oneline -20` in that repo shows 20 commits, most recent `ba0e580 Add combined steering/accel bars...`, `e339901`, `b244b2e`, `79ff129`, `68055d3 Merge MPC controller nodes into one; sync chatter-fix/anti-hunt backlog` (Aug 31), and `102c249 Updated PR changes md file.` (Aug 17) — i.e. everything in this doc's "pending" list was committed by Aug 31 at the latest, three weeks after this doc's 2026-08-13 date.
- There is now a **new** round of uncommitted working-tree changes on top of that (as of today, 2026-09-29): `mpc_controller.py`, `mpc_core.py`, `nmpc_core.py`, `nmpc_params.py`, `control_utils.py`, `live_viz.py`, `stanley_controller.py`, `telemetry_logger.py`, `setup.py`, `README.md`, `fsae_params.yaml`, plus several track CSVs — none of which match this doc's Aug-13 list (that list's content is long since merged in).
- "no upstream drift" cannot be confirmed without a `git fetch` (explicitly not run per audit constraints), but local `git log` shows the branch itself changed identity (main → a feature branch) since this doc was written, which the doc does not reflect at all regardless of fetch status.

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 3 | "uncommitted working-tree changes... as of 2026-08-13" | STALE | that Aug-13 working tree was committed across commits up to `68055d3` (Aug 31); current uncommitted set is a different, later batch of files | rewrite header, drop the 2026-08-13 framing, either regenerate against current `git status`/`CHANGES.md` tip or mark doc as an dated historical snapshot |
| 3 | "Local main is up to date with origin/main (no upstream drift)" | WRONG (branch premise) | current branch is `feature/nmpc-and-controller-improvements`, not `main` | correct to name the actual branch; note that a real drift check needs `git fetch`, not done here per audit scope |
| 3 | "the live repo is out of scope for edits/commits from here" | OK, consistent with CLAUDE.md | CLAUDE.md: "fsae_planning: read-only for agents... Never commit or push here" with the narrow exception only for the named feature branch and only with explicit per-instance user go-ahead | none — framing matches the hard rule |
| 46 | `mpc_controller_standalone.py` merged into `mpc_controller.py` as `standalone_output=true` mode | OK (mechanism), STALE (framing as "pending") | live `mpc_controller.py` module docstring: "This node has TWO output modes, selected by the `standalone_output` ROS2 parameter"; `find . -iname mpc_controller_standalone*` only finds a stale `.pyc`, no `.py` source | mechanism claim is correct and now long since committed (`68055d3`), not "pending" |
| 35 | `nmpc_core.py`/`nmpc_params.py` exist | OK, but path stale | both exist at `control/fsae_control/fsae_control/mpc/nmpc_core.py` / `.../mpc/nmpc_params.py` — doc doesn't give an exact path so not wrong, but note they now live under a new `mpc/` subpackage (see doc 3's line-number findings) | low priority, doc doesn't claim a specific path |
| 52 | `scoring.py` exists as new file | OK | `control/fsae_control/fsae_control/scoring.py` present | none |
| 53 | `cone_recorder.py`/`.launch.py` exist | OK | `perception/fsae_sim_perception/fsae_sim_perception/cone_recorder.py` + `common/fsae_bringup/launch/cone_recorder.launch.py` present | none |
| 54 | `tracks/comp_test_map_3/` has cone map + both exported CSVs | OK, actually richer | `ls tracks/comp_test_map_3/` shows `cone_map.json`, `speed_profile.csv`, `raceline.csv`, `centerline.csv`, plus an extra `speed_profile_corner_test.csv` (untracked, per `git status`) not mentioned by the doc, expected since that's newer work | none needed, doc's claim is a subset of what's there, not contradicted |
| 55 | `control/fsae_control/test/nmpc_offline_check.py` exists | OK | file present at that exact path | none |
| 66 | `LapProgressTracker` class exists | OK | `telemetry_logger.py:376: class LapProgressTracker:` | none |
| 67 | `ADAPTIVE_COLUMNS` constant exists and lists exactly what `compute()` writes | OK (existence); content plausible | `ADAPTIVE_COLUMNS` defined in `telemetry_logger.py`, first entries match doc's description of the corner-factor scheduler columns (`corner_factor`, `low_speed_corner_boost`, `corner_frac`) | none, deeper content-diff not performed (out of scope) |
| 39 | field count "44 as of the corner-factor rewrite, down from ~56" | UNVERIFIABLE without counting live `MPCParams` dataclass fields today, and CLAUDE.md's own count (106, combined `MPCParams`+`NMPCParams`) is a different, larger, more recent number for a different scope (both dataclasses) | doc's 44/56 concerns `MPCParams` alone at an earlier point; CLAUDE.md's 106 is the combined 2026-09-29 count including `NMPCParams` — not necessarily contradictory, just not cross-checked field-by-field | flag as needing a recount if this doc is kept |

**Mechanism-without-why:** the doc mostly states why for each bullet (this is one of its strengths — nearly every line has a "the old version could X" justification). No major gaps found here.

**Code paths/modules mentioned:** `sim_perception.py` (implicit), `boundary.py`/`cone_map.py` implicit via "Planning", `control_utils.py`, `mpc_core.py`, `mpc_params.py`, `mpc_controller.py`, `mpc_controller_standalone.py` (now gone), `nmpc_core.py`, `nmpc_params.py`, `scoring.py`, `cone_recorder.py`, `telemetry_logger.py`, `setup.py`, `control.launch.py`, `sim.launch.py`, `fsae_params.yaml`, `tracks/comp_test_map_3/`, `control/fsae_control/test/nmpc_offline_check.py`.

**Readability:** good, concise bullets, matches CLAUDE.md's doc-writing-style rules (one idea per bullet, why explained). Main problem is purely temporal staleness, not structure.

**Proposed action:** **rewrite/regenerate, don't delete.** The doc's format (a curated "what's pending toward `fsae_planning`" tracker) is exactly the kind of thing CLAUDE.md's "change ledger" framing wants, but its content is a snapshot frozen at 2026-08-13 that has since been entirely committed and superseded by a new, different pending set. Recommend: either (a) regenerate it fresh against current `git -C ros2/src/fsae_planning status`/`log`/`diff` and the actual current branch name, or (b) rename it to something like `fsae_planning_pending_pr_2026-08-13.md` and move to `docs/logs/` as a dated historical snapshot, then write a fresh pending-PR doc for the current uncommitted set. Given CLAUDE.md's "docs/logs are curated history, not a raw ledger" framing and this doc's stated purpose as a live tracker, (a) regenerate in place is likely intended, not (b).

---

## Doc 3: `docs/steering_turn_in_upgrade_options.md`

**Purpose/audience:** an options-analysis/investigation doc about NMPC steering-rate weighting for late/jerky turn-in on shallow corners. Audience: whoever picks up the turn-in problem next.

**Overlap:** heavily overlaps with `docs/logs/steering_chatter_investigation.md` (referenced at that doc's line 162, and is itself downstream of the `r_rate_delta` chatter fix from that log) and with `docs/tuning.md`'s "Input-jerk cost" section, which documents the shipped, superseding fix.

### Headline finding: this document is fully superseded and should be archived

The doc's own git history (`git log --follow --oneline`) shows it was written and iterated through Option 1/2's live rejections:

```
a3782d4 Analyse late/jerky shallow-corner turn-in; document upgrade options
5b7c9c6 Configure Option 2 (curvature-scheduled R_rate) for live test
36ee99f Correct status header: Option 2 is configured, not analysis-only
8512e2c Option 2 live-tested and rejected; revise recommendations
6d4fb37 Add per-stage steering-rate ramp (Option 1); offline-rejected as a jerk fix
fccc739 Option 1 live-tested and rejected; reverted to flat r_rate_delta=52.5
```

...but its **status header (line 3) was never updated** after that: it still reads "Status: Option 2 is CONFIGURED and awaiting a live test," even though the body text below it (lines 112-126, "LIVE-TESTED AND REJECTED") shows Option 2 was tested and rejected, and the "Recommended sequence" section (lines 184-191) correctly says Option 2 is "done, rejected" and Option 4 is "the only remaining candidate... Do this next." **The one-line status banner at the top contradicts the doc's own body**, an internal staleness bug independent of anything that happened after this file's last edit.

More importantly, **Option 4 (the doc's own final recommendation) has since been implemented and shipped**, confirmed by:

- `docs/tuning.md` lines 228-254 document `nmpc_rjerk_delta`/`nmpc_rjerk_a` as a real, tunable, shipped feature with live/offline defaults **150.0 / 0.0**.
- `settings.py` (offline): `NMPC_RJERK_DELTA = 150.0`, `NMPC_RJERK_A = 0.0` (exact match).
- Live `nmpc_core.py`'s `_build_qp()` (around line 1234-1287) builds a real second-difference operator: `self._E2 = E @ E`, `self._E2rE2 = ...`, and the solve step (line ~1779-1789) folds `e_jerk = self._E2 @ u_flat` into the Hessian/gradient — this is exactly the "second-difference penalty," "changes the QP's Hessian sparsity pattern" implementation the doc's Option 4 section (lines 149-168) describes as future work.
- Both `ros2/launch_all.sh` and its mirror `fsds_simulator/launch_all.sh` currently ship `NMPC_RJERK_DELTA=150.0` **enabled** (uncommented), while `NMPC_CORNER_RRATE_BLEND_ENABLED=false` (Option 2, this doc's "configured" status) is **disabled**, directly contradicting the doc's status line.
- Commit `ad469dfc` ("Make the new NMPC rate-shaping fields tunable...", Aug 24) shows `rjerk_delta` already promoted to a tuned, offline-tuner-searched parameter three weeks before this doc's last-modified date and five weeks before today.
- A **third, newer mechanism this doc never mentions at all** now also ships by default: the "three-zone" `NMPC_RRATE_ZONE_*` schedule (`NMPC_RRATE_ZONE_ENABLED=true`, boost/ease/floor by horizon position and lookahead), layered on top of and composing with the rjerk term per `launch_all.sh`'s own comments ("Composes multiplicatively with NMPC_RJERK_DELTA below, which is also on"). This doc's Option 1 vs Option 4 framing is now a two-generations-old picture of the design space.

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 3 | "Status: Option 2 is CONFIGURED and awaiting a live test" | WRONG / STALE | body text (112-126) shows Option 2 live-tested and rejected; `launch_all.sh` ships it `false`; `NMPC_RJERK_DELTA` (Option 4) is the one actually shipped `true` | rewrite status header, or archive whole doc (see recommendation) |
| 30 | `nmpc_core.py:1110` builds rate weight `np.tile(self.r_rate, N)` uniform across horizon | STALE (line number), mechanism itself superseded | live `nmpc_core.py` line 1110 today is a comment about the "steering-rate cost... boost on a true straight, ease on approach... floor through the corner" (the three-zone schedule) and rjerk weight assembly, not a flat `np.tile` call; the file has grown/reorganized since this doc's line numbers were recorded (2364 lines currently) | drop specific line numbers or regenerate against current file; note the "uniform across horizon" premise is now false since both rjerk and rrate-zone are live and non-uniform |
| 43-48 | code snippet "nmpc_core.py, replacing np.tile(self.r_rate, N)" | STALE mechanism reference | the ramp described (`nmpc_rrate_stage_ramp_enabled`) was in fact implemented, exactly as later text in the same doc (line 57) says, in `controller/nmpc/weight_schedule.py`'s `_rrate_stage_ramp()` (offline) and inline in live `nmpc_core.py` (function present at line 910 in the live 2364-line file today) | none needed for the mechanism claim (doc itself documents the implementation later), but the line-45-ish inline code snippet is presented as a proposal when it's actually already-implemented-and-rejected by the time a reader reaches line 55 of the same doc — a readability/ordering problem, not a factual error |
| 50 | "`_ErE`/`_Rr_flat` are already recomputed per tick... (`nmpc_core.py:1627`)" | STALE (line number) | live line 1627 today is inside `_report_costs`/output-term dict construction ("out_terms = {...}"), unrelated to `_ErE` recompute plumbing | update or drop line number; the underlying claim (plumbing exists) is still true, evidenced by `self.rrate_stage_ramp_enabled` being read from `pm.nmpc_rrate_stage_ramp_enabled` in the live file, just not at that line anymore |
| 164 | "`E` is built once in `_build_qp` (`nmpc_core.py:848-855`)" | STALE (line number) | live lines 848-855 today are inside `H[:, 2] = e_psi` / progress-term / friction-circle-term assembly (a different, unrelated block, from the progress-term / friction-circle features added later); `_build_qp` itself is now at line 1234, and `self._E = E` is at line 1272 | update line references; note the doc's own prediction that this would "need... the OSQP P matrix setup... rebuilt" is confirmed correct by the current code (`self._E2rE2` is a distinct Hessian contribution folded in conditionally) |
| 50, 57, 92-104 | `nmpc_rrate_stage_ramp_enabled`/`nmpc_rrate_stage_near`, `_rrate_stage_ramp()`, `controller/nmpc/weight_schedule.py` | OK (paths current) | `controller/nmpc/weight_schedule.py` exists offline with `def _rrate_stage_ramp(N, near):` at line 59; live `nmpc_core.py` has matching `_rrate_stage_ramp` at line 910, and `self.rrate_stage_ramp_enabled = bool(pm.nmpc_rrate_stage_ramp_enabled)` at line 1101 | none, these names/paths are current and correct despite the file having moved/split elsewhere |
| 96-104 | `NMPC_CORNER_RRATE_BLEND_ENABLED`, `NMPC_RRATE_STEER_STRAIGHT`, `NMPC_RRATE_STEER_CORNER` fields exist and are the config to try | OK (fields exist and are wired), STALE (recommendation to "try this first") | fields present in `settings.py` (`NMPC_CORNER_RRATE_BLEND_ENABLED = False`, `NMPC_RRATE_STEER_STRAIGHT/_CORNER = -1.0`) and in both `launch_all.sh` copies, currently shipped **disabled**, superseded by the rjerk/zone mechanisms | mark as historical, not a live "try this" instruction |
| 174 | 180°/s slew is "a measured lower-bound estimate," see `docs/reference/README.md`'s slew-rate section | mechanism value OK, anchor WRONG | live `du_max = math.radians(180.0) * self.dt` confirmed at `nmpc_core.py:1045`, matches doc 2's own claim of "80°/s → 180°/s" history; but `docs/reference/README.md` has no such section (see Doc 1 findings above), it's a dangling anchor into an index-only file | fix anchor to whichever subject file (`control_mechanisms.md` or `offline_live_parity.md`) actually documents the slew-rate measurement |
| whole doc | Option 4 = "the only remaining candidate... Do this next" | SUPERSEDED (confirmed acted on) | `nmpc_rjerk_delta=150.0`/`nmpc_rjerk_a=0.0` shipped in both `settings.py` and live `mpc_params`/`nmpc_params`, documented as a real shipped feature in `docs/tuning.md`, wired in `launch_all.sh` (both copies), and implemented with a real `E2`/Hessian addition in `nmpc_core.py` | this doc's entire remaining-work recommendation has been executed; doc should be archived/superseded, see action below |

**Mechanism-without-why:** doc is generally strong on "why" (root-cause section, quantified ratios, live-log evidence). No significant gaps.

**Readability:** good structurally (tables, headers, bolded key numbers), consistent with CLAUDE.md's writing-style rules. The one readability problem found: the status header (line 3) is now actively misleading relative to the doc's own body and to reality, which is worse than a merely-stale doc because a skimming reader who reads only the header (as the writing-style guide itself recommends: "a reader who stops after two lines should still have the finding") gets the wrong answer.

**Code paths/modules mentioned:** `nmpc_core.py` (live, now split conceptually across a monolithic 2364-line live file vs. offline `controller/nmpc/{layout,outputs,reference,dynamics,solver,weight_schedule}.py` package), `controller/nmpc/weight_schedule.py`, `docs/logs/steering_chatter_investigation.md`, `tuner.steering_chatter_check`, `peak_kappa_ahead()` (removed), `curvature_speed()`.

**Does `docs/logs/late_turn_in_investigation.md` document Option 4's resolution?** No direct textual overlap found: a grep for "rjerk"/"Option 4"/"steering acceleration" in `late_turn_in_investigation.md` returns nothing. The rjerk/jerk-cost work appears to live in its own commit history and in `docs/tuning.md` directly, not narrated in that particular log. (`late_turn_in_investigation.md` covers a different, earlier investigation thread — heading-lead/NMPC-formulation work, Parts 8-17 — not this specific chatter/turn-in options doc's lineage.) The actual superseding narrative lives in `docs/logs/steering_chatter_investigation.md` (which doc 3 itself cites for the `r_rate_delta` root cause) and in this doc's own commit history plus `docs/tuning.md`.

**Proposed action: archive/mark superseded, do not silently delete.** Every one of its four numbered candidate options has been resolved (1 and 2 rejected offline+live, exactly as later parts of the same doc record; 3 deprioritized by the doc's own reasoning; 4 implemented and shipped as `nmpc_rjerk_delta`/`nmpc_rjerk_a`, now documented as current in `docs/tuning.md`). Recommend:
1. Update the status header to something like "SUPERSEDED (2026-09-29): Option 4 implemented and shipped as `nmpc_rjerk_delta`/`nmpc_rjerk_a`, see `docs/tuning.md`'s Input-jerk cost section. Options 1-3 rejected, kept here for the negative result."
2. Move the file to `docs/logs/` (it is now a closed investigation with a decided outcome, matching CLAUDE.md's own description of what belongs in `docs/logs/`) rather than living among the active/current docs.
3. Do not delete outright: its quantified rejection evidence for Options 1-3 (offline+live numbers) is exactly the kind of "falsified, with the numbers" result CLAUDE.md's writing-style section says must not be lost, since a null result nobody can find gets re-tested. Its own text already anticipates this ("Kept in the code behind a default-off flag... may matter for restoring `nmpc_offline_check`").
4. Separately fix the stale line-number citations (`nmpc_core.py:1110`, `:1627`, `:848-855`) if the doc is kept in any active form, since the live file has grown to 2364 lines and reorganized around progress-term/friction-circle features added after this doc was written; regenerate line numbers or drop them in favor of function-name references (which do still resolve correctly, e.g. `_build_qp`, `_rrate_stage_ramp`).

---

## Summary table (all 3 docs)

| doc | OK | STALE | WRONG | UNVERIFIABLE |
|---|---|---|---|---|
| planning_control_sync.md | 6 (all link targets) | 0 | 0 | 0 |
| fsae_planning_pending_pr.md | 8 | 2 (header framing, standalone_output "pending") | 1 (branch/up-to-date claim) | 1 (field count) |
| steering_turn_in_upgrade_options.md | 3 (field/path names) | 4 (3 line-number citations + dangling anchor) | 2 (status header, "Option 2 configured") | 0 |

