# Sophia add-on — use on your feature branch

Use **after** the shared foundation ZIP has been copied to your GitHub repository on `main`. Unzip this archive and copy the CONTENTS of `02_Sophia_Addon` to the repository root while your `feature/sophia-prediction-analysis` branch is checked out. Never commit the ZIP itself.

## Test in repository root

```bash
python -m pip install -r requirements.txt
python -m pytest -q
python scripts/sophia_smoke.py
python scripts/sophia_analyze.py --input results/kinematic/summary.csv --output results/sophia_kinematic --backend kinematic --expected-pairs 20
```

This input summary file is from the **simplified kinematic Python simulator**, not Webots.

The file `main.tex` is a full copy/paste-ready report document. In Overleaf, start a new blank project, replace `main.tex` with its contents and upload the two figure files to `figures/`. Compile using pdfLaTeX. Without figure files the source shows placeholders. The report contains preliminary kinematic results only and explicit Webots TODOs.

Do NOT merge your branch until Anastasiia reviews the observation contract and checks her Webots output against `docs/SOPHIA_HANDOFF.md`.
