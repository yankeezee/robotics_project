# Integrationsguide for Sophia och Anastasiia

**Status:** Forberedd och testad i Python, **inte** kordin Webots. Denna mapp ar ett integrationsforslag, inte en redan inlamningsklar slutversion. Bevara bada originalbrancherna ororda.

## Vad som faktiskt integrerats

- Anastasiias `pedestrian_navigation/navigation/planning.py` ar den **gemensamma** velocity-sampling-planeraren.
- `ReactivePlanner` behaller fotgangarpositionerna fasta under utrullningen.
- Nya `PredictivePlanner` anropar **Sophias oforandrade** `pednav/sophia/estimation.py` for ID-baserad hastighetsestimering och extrapolerar de uppmatta positionerna.
- Samma begransningar, cost function, passeringsheuristik, stoppvillkor, statiska rutt, nodscenarier och konservativa geometriska collisionscheckar anvands for bada metoderna.
- Webots-controller valjer planerare via `--planner reactive` eller `--planner predictive`.
- Aktuell observationsdata innehaller stabila fotgangar-ID, men aldrig forhandsinformation om simulerade fotgangarbanor.
- Analysverktyget kan tolka **faktiska** Webots-resultat i Sophias CSV-format. Inga nya Webots-resultat finns i paketet.

## Rekommenderad GitHub-process

1. Sakerstall att Sophias och Anastasiias separata brancher ar pushade och att `main` har direkt placerade `pednav/`, `requirements.txt` och `README.md`.
2. Skapa en **ny** branch `integration/webots-prediction` utgaende fran uppdaterad `main`.
3. Mergea sedan de tva feature-brancherna till integrationsbranchen, **inte** till `main`. Granska eventuella konflikter i `README.md` och den gamla undermappen `PedestrianNav_Shared_Foundation/` utan att kasta bort nagon persons kod. Om detta ar krangligt, bevara branches separat och anvand review-snapshoten som referens for manuell integration.
4. Kopiera filerna i `Integration_Overlay/` over integrationsbranchens projektrot (inte sjalva mappen `Integration_Overlay`). Granska med `git diff` innan commit.
5. Kor tester lokalt och be Anastasiia granska Webots-delen, Sophia estimatorn och gemensamma parametrar innan ni mergear till `main`.

**OBS:** GitHub Desktop kan anvandas for branch och merge. Om en konflikt uppstar: avbryt hellre och be om hjalp an att valja `Accept all changes` pa mafa.

## Mac: lokal kodkontroll

I repositoryts **rot**:

```bash
python3 -m pip install -r requirements.txt
python3 -m pytest -q
python3 scripts/sophia_smoke.py
```

For separata Webots-test i projektmappen:

```bash
cd pedestrian_navigation
python3 -m unittest discover -s tests -v
python3 run.py --planner reactive --scenario crossing --seed 3 --show
python3 run.py --planner predictive --scenario crossing --seed 3 --show
```

`run.py` letar efter `webots` pa PATH och den vanliga Mac-installationen `/Applications/Webots.app/Contents/MacOS/webots`. Vid behov ange explicit `--webots "/Applications/Webots.app/Contents/MacOS/webots"`.

**Webots-installation:** varlden ar definierad for Webots R2025b. For en direkt oppnad `.wbt` behover Webots Python-miljo samma beroenden, framfor allt NumPy for prediktion. Den reaktiva metoden anvander enbart standardbiblioteket.

## Parvis utvecklingskontroll

Borja med seed 3 som forsta **nytt utvecklingsfall**. Kor bada metoder i Webots, notera verkliga loggmappar och kontrollera visuellt motorernas riktning, att roboten nar malet samt att resultaten i `summary.json` ar rimliga. Skriv inte over tidigare utvecklingsloggar.

Nar ni ar redo och har fryst samma parametrar, anvand **nya** sluttestfron som inte anpassats med tiden (exempelvis 6000--6019 om dessa fortfarande ar ororda):

```bash
python3 tools/run_pairs.py --seed-start 6000 --trials 20
```

Detta startar Webots 40 ganger och kor varje seed med de tre familjerna i bada planerarlagena. Det producerar **forst efter verkliga Webots-korningar** `results/webots_paired_summary.csv`.

Efterat, fran repositoryts **rot**:

```bash
python3 scripts/sophia_analyze.py \
  --input pedestrian_navigation/results/webots_paired_summary.csv \
  --output pedestrian_navigation/results/webots_analysis \
  --backend webots --expected-pairs 20
```

Detta validerar 60 par och genererar rapporttabeller och figurer. Om nagra forsok saknas avslutas analysen med fel. Spara `run_config.json`, `summary.json`, `trajectory.csv`, `observations.jsonl`, `source_manifest.json` och `webots.log` for varje korning.

## Parametrar och viktiga beslut

Utgangspunkt: Anastasiias Webots-inställningar (`Parameters` i `navigation/core.py`): dt 0.064s, horisont 2.0s, robotradie 0.14m, fotgangarradie 0.25m, max 0.20m/s, acceleratorgrans 0.35m/s2, sakerhetsmarginal 0.20m. Detta motsvarar **inte** de ursprungliga Sophia-kinematiska resultaten (3s, andra radier och hastigheter). Jamfor aldrig resultaten rakt av som samma experiment.

Sophias estimator behaller 4 tidsstamplade observationer per person och begransar skattad fart till 0.8m/s. Historiken startar om for varje nytt forsok. Aktuella observationer skickas bada, men framtida scriptbanor skickas ingen av planerarna.

**Kvarstar:** Webots-kompatibilitet pa er dator; flerfotgangarscenarier (Anastasiias medfoljande kod har bara en fotgangare per scenario); slutlig parameterfrysning; 60 par med verkliga Webots-loggar; slutrapportens Webots-resultat och demo.
