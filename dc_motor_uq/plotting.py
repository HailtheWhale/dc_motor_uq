import numpy as np
import matplotlib.pyplot as plt

from .utils import BASE_DIR
from .model import run_single_sample, MotorDC

def solver_w_plot(motor:MotorDC, 
                  title:str):
    
    save_dir = BASE_DIR / "Images"
    save_dir.mkdir(exist_ok=True)

# Getting the plots for the motor speed, bc PID ctrls this. 
# Ensures NaNs do not break plots. 
    w_mean     = np.nanmean(motor.w, axis=0)
    w_std      = np.nanstd(motor.w, axis=0)
# Envelope of uncertainty, 90% credible band.
    w_bnd_low  = np.percentile(motor.w, 5, axis=0)
    w_bnd_high = np.percentile(motor.w, 95, axis=0)

# Plotting
    fig, ax = plt.subplots()
    # Probability Envelope. 
    ax.plot(motor.t_eval, w_mean, label="Mean Response", color="blue")
    ax.fill_between(motor.t_eval, w_bnd_low, w_bnd_high, color="orange", alpha=0.3, label="90% Envelope")
    # Deterministic Trajectory
    det_params = np.median(motor.params, axis=0)
    det_i, det_w = run_single_sample(
        (det_params, [0,0], motor.t_eval, motor.target_w, [motor.Kp, motor.Ki, motor.Kd])
        )

    ax.plot(motor.t_eval, det_w, label="Nominal", color="black", linestyle="--")
    ax.legend()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("\u03C9 (rad/s)")
    ax.grid()
    ax.set_title(f"{title}\nMean Response with 90% Uncertainty Envelope")
    fig.savefig(save_dir/ f"{title}.png", dpi=300,bbox_inches="tight")
    plt.close(fig)

def solver_histo_plot(motor:MotorDC, 
                      title:str):
    
# Directory Setup 
    save_dir = BASE_DIR / "Images" / "Histograms"
    save_dir.mkdir(exist_ok=True)

# Retrieve the performance metrics analyzing
    motor.get_performance_metrics()
    metrics = [motor.times_rise, motor.times_settle, motor.percent_overshoots, motor.ss_errs]
    labels = ["Rise Time (s)",
            "Settling Time (s)",
            "Percent Overshoot (%)",
            "Steady-State Error (rad/s)"]
    
    # drop NaNs during plotting.
    cleaned_metrics = [m[~np.isnan(m)] for m in metrics]

    # Report number of NaNs occurred for debug.
    for label, m in zip(labels, metrics):
        n_nan = np.isnan(m).sum()
        if n_nan > 0:
            print(f"{label}: {n_nan} samples produced NaN values. Excluded these from histogram.")

    for data,label in zip(cleaned_metrics, labels):
        fig, ax = plt.subplots()

        ax.hist(data, bins=30, color="blue", edgecolor="black")
        ax.set_xlabel(label)
        ax.set_ylabel("Count")
        ax.set_title(title)

        # Clean the filename
        safe_label = label.replace("/","_").replace("(","").replace(")","").replace(" ","_")
        fig.savefig(save_dir / f"{safe_label}_{title}.png", dpi=300, bbox_inches="tight")
        plt.close(fig)
