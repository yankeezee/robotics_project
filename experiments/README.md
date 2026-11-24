# Webots experiment artifacts

All directories distinguish development diagnostics from the prospectively
frozen paired comparison. None of these results are from the Python-only
kinematic simulator.

| Directory | Role |
| --- | --- |
| [`webots_integration_diagnostics_2026-11-24/`](webots_integration_diagnostics_2026-11-24/README.md) | Ten original development diagnostics, both modes, seed 3; includes empty/static controls |
| [`webots_integration_smoke_2026-11-24/`](webots_integration_smoke_2026-11-24/RESULTS.md) | Six isolated development runs, three paired cases, with integrity audits |
| [`webots_paired_2026-11-24/`](webots_paired_2026-11-24/RESULTS.md) | Completed prospective evaluation on seeds 6000–6019: 60 pairs / 120 runs |

The common protocol is documented in
[`docs/WEBOTS_EXPERIMENT_PROTOCOL.md`](../docs/WEBOTS_EXPERIMENT_PROTOCOL.md).
The paired experiment's `protocol.json`, `parameters.json`, `cases.json`, and
`frozen_source.zip` are created before its first trial. `progress.json` and
`summary.csv` are checkpoints; use the completed `RESULTS.md` and analysis
outputs for interpretation after collection is complete.

Raw logs are kept locally under `raw/` and published in `raw_logs.zip` after
analysis. Extract that archive into its experiment directory to reconstruct the
raw tree on another machine. `artifact_manifest.json` records hashes for both
the compressed bundle and individual files. Historical absolute output paths
are provenance metadata; reproduce with the documented launcher arguments and
the archived parameter/source versions.
