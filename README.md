# Single-UAV Mission Planning with a Genetic Algorithm

**Artificial Intelligence – BSCS 5 · Assignment 2 · Bahria University Lahore Campus**
**Author:** [hariswasim617](https://github.com/hariswasim617) – [Roll No]

A Genetic Algorithm (written from scratch in Python) that finds the **deployment position, waypoint
route, speeds and sensor activation schedule** of **1 drone** surveying a **50 x 50 hazard grid**
with no-fly zones, hazard zones (wind / radiation), disaster (priority) zones and targets.

![Flight paths across generations](results/paths_generations.png)

## How to run (Google Colab only)
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/hariswasim617/AI-Assignment-2/blob/main/UAV_GA_Colab.ipynb)

1. Click the badge above, **or** go to [colab.research.google.com](https://colab.research.google.com) →
   *File → Upload notebook* → choose `UAV_GA_Colab.ipynb` from this repository.
2. Click **Runtime → Run all**. Nothing needs to be installed – Colab already has numpy and matplotlib.
3. The figures appear under each cell (about 5 minutes in total) and are saved in the `results` folder.

The notebook contains the same code as the `.py` files, one file per cell. The `.py` files are kept in the
repository so each part of the code can be read on its own.

To try other settings, change the numbers in the **Settings** cell (`config.py`) and run all cells again.

## Files
| File | What it does |
|---|---|
| `config.py` | all settings (map, drone, fitness weights, GA settings) |
| `environment.py` | the grid: no-fly zones, hazard zones, priority zones, targets |
| `chromosome.py` | `Chromosome` class: deployment position + waypoints, speed, sensor power and sensor on/off of the drone |
| `fitness.py` | multi-objective fitness function + constraint handling (repair and penalty) |
| `ga.py` | `GeneticAlgorithm` class: population initialisation, tournament selection, crossover, mutation, termination |
| `random_search.py` | baseline random search for the comparison |
| `plots.py` | all figures |
| `main.py` | the experiment functions (main run, sensitivity, GA vs. random search); the notebook calls them |
| `results/` | figures (PNG) and result tables (CSV) |
| `report/` | technical report (Word + PDF) |
| `UAV_GA_Colab.ipynb` | the whole project as a Google Colab notebook (same code, one file per cell) |

## Design
**Chromosome** (hybrid real + binary, 2 + 6 x 5 = 32 genes):
```
[ deploy_x, deploy_y | (x, y, speed, power, on) x 6 waypoints ]
```
`deploy_x, deploy_y` = where the drone is launched and lands, `x, y` = waypoint, `speed` = speed on the
leg to that waypoint, `power` = sensor power (radius 1–4 cells), `on` = sensor on/off on that leg.
The drone flies deployment position → 6 waypoints → deployment position.

**Fitness** (every part scaled to 0..1):
```
F = 0.35*Coverage + 0.25*Detection - 0.15*Energy - 0.10*Time - 0.15*Hazard - 2*Penalty
```
* maximise: priority-weighted coverage of the disaster zones, target detection rate
* minimise: energy, flight time, hazard exposure
* penalty: flying through a no-fly zone or further than the maximum range (150 cells)

**Constraint handling** – repair (the deployment position / waypoints inside a no-fly zone are moved to the
nearest open cell) + penalty function.

**Genetic operators** – tournament selection (k = 3) with elitism (2); crossover = one-point route crossover
(first waypoints from one parent, the rest from the other) + geometric blend; mutation = Gaussian move of
waypoints / the deployment position, small speed and power
changes, sensor bit flip and waypoint swap; 2 random immigrants per generation keep the population
diverse. **Termination:** 100 generations or 30 generations without improvement.

## Results
**Main run** (population 50, 100 generations, seed 42): best fitness **-0.056 → 0.360**.
The final plan covers 0.69 of the priority-weighted area, finds 100% of the targets and breaks no rules.

| Fitness convergence (best vs. average) | Coverage heatmaps across generations |
|---|---|
| ![](results/convergence.png) | ![](results/coverage_heatmaps.png) |

**Hyperparameter sensitivity** (final best fitness, mean ± std over 5 seeds):

| Setting | Value 1 | Value 2 | Value 3 |
|---|---|---|---|
| Population size | 20: 0.190 ± 0.075 | 50: 0.309 ± 0.075 | 100: 0.286 ± 0.077 |
| Mutation rate | 0.02: 0.264 ± 0.072 | 0.1: 0.309 ± 0.075 | 0.3: 0.182 ± 0.062 |
| Selection pressure (tournament k) | 2: 0.237 ± 0.085 | 3: 0.309 ± 0.075 | 5: 0.223 ± 0.076 |

With one drone some runs get stuck on a plan that covers only one disaster zone (fitness about 0.15–0.17),
so the standard deviations are large; see the report for the discussion.

![Sensitivity](results/sensitivity.png)

**GA vs. random search** (same number of fitness evaluations, 5 seeds):

| | GA | Random search |
|---|---|---|
| Fitness | **0.309 ± 0.075** | 0.083 ± 0.014 |
| Coverage | 0.600 ± 0.136 | 0.278 ± 0.051 |
| Detection | 0.893 ± 0.213 | 0.427 ± 0.080 |
| Energy | 0.466 ± 0.114 | 0.407 ± 0.084 |
| Time | 0.330 ± 0.101 | 0.423 ± 0.129 |
| Hazard | 0.142 ± 0.024 | 0.116 ± 0.028 |
| Penalty | 0.000 ± 0.000 | 0.000 ± 0.000 |

![GA vs random](results/ga_vs_random.png)
![GA vs random maps](results/ga_vs_random_maps.png)

The full discussion is in [`report/Technical_Report.pdf`](report/Technical_Report.pdf).
