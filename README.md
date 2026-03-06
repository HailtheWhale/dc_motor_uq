# dc_motor_uq: DC Motor Uncertainty Quantification Package
### Simulation, control, and uncertainty analysis for a DC motor model

`dc_motor_uq` is a modular Python package for simulating a DC motor under parametric uncertainty. It models the motor dynamics, applies a PID controller, and quantifies how variations in physical parameters affect time‑domain performance metrics such as rise time, overshoot, and settling time. The package includes dedicated modules for modeling, sampling, metrics, and visualization, along with a runnable example script (`scripts/run_simulation.py`) that demonstrates the full simulation pipeline - parameter sampling, parallel model execution, metric computation, and visualization - with a modular package architecture. 
