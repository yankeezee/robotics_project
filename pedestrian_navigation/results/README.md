# Versioned development evidence

These selected historical run directories support the README and research log.
They include the original failures, intermediate revisions, final seed-0 Webots
checks, additional seed-2 Webots checks, and referenced kinematic experiments.

Historical launch configurations may contain the original machine's absolute
output paths. They are provenance records, not portable launch commands. To
reproduce a case, use `python run.py --scenario <family> --seed <seed>` from
`pedestrian_navigation/`, and use the archived parameter/source snapshot when
reproducing an older controller revision.

New timestamped runs are ignored by default; the explicitly retained evidence
directories are listed in the local `.gitignore`. High-resolution `scene.png`
captures are omitted, while the referenced compact `scene_preview.png` is kept.
Raw collision outcomes and margin violations are preserved.
