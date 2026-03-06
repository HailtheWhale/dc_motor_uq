# dc_motor_uq: DC Motor Uncertainty Quantification Package
### Simulation, control, and uncertainty analysis for a DC motor model

`dc_motor_uq` is a modular Python package for simulating a DC motor under parametric uncertainty. It models the motor dynamics, applies a PID controller, and quantifies how variations in physical parameters affect time‑domain performance metrics such as rise time, overshoot, and settling time. The package includes dedicated modules for modeling, sampling, metrics, and visualization, along with a runnable example script (`scripts/run_simulation.py`) that demonstrates the full simulation pipeline — parameter sampling, parallel model execution, metric computation, and visualization — with a modular package architecture. 

## Project Overview
This project analyzes how uncertainty in physical parameters affects the closed‑loop behavior of a DC motor. The package separates the workflow into clear components—modeling, sampling, simulation, metrics, and visualization—mirroring how real autonomy and controls software is structured. By running many perturbed simulations in parallel, the package highlights how variations in resistance, inductance, inertia, and damping influence key performance metrics such as rise time, overshoot, and settling time. The example script ties these components together into a single workflow that demonstrates uncertainty‑aware actuator analysis in a compact, extensible format.

## Key Features

- **Modular architecture**  — Separate modules for modeling, sampling, simulation, metrics, and visualization.
- **Uncertainty quantification workflow** — parameter sampling, parallel execution, and distribution‑based performance analysis.
- **PID‑controlled motor model** — standard electromechanical dynamics with configurable gains.
- **Performance metrics** — rise time, overshoot, settling time, steady‑state error, and more.
- **Visualization tools** — time‑domain plots, histograms, and distribution summaries.
- **Runnable example script** — end‑to‑end demonstration of the full simulation pipeline using `scripts/run_simulation.py`.

## Package Structure

```
dc_motor_uq/
├── model.py              # Motor dynamics and PID controller
├── sampling.py           # Parameter sampling utilities
├── simulation.py         # Simulation loop and parallel execution
├── metrics.py            # Time-domain performance metrics
├── visualization.py      # Plotting utilities
scripts/
└── run_simulation.py     # End-to-end example workflow
```

## Installation
Clone the repository and install the required Python packages:
```
git clone https://github.com/HailtheWhale/dc_motor_uq.git
cd dc_motor_uq
pip install -r requirements.txt
```
