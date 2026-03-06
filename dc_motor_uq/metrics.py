import numpy as np

def compute_metrics(t:np.ndarray, 
                    n_samples:int,
                    w:np.ndarray, 
                    target:np.float64):

# Getting Performance Metrics. Each sample taken will have these associated 
# With them. 
    times_rise         = []
    times_settle       = []
    percent_overshoots = []
    ss_errs            = []

    # How many times signal did not settle
    n_not_settled = 0

    for i in range(n_samples):
        w_i   = w[i,:] # Pull the RPMs for a given sample
        w_max = np.max(w_i)    # Find the max value
    # If the max val is more than the target, we have overshoot. 
        if (w_max > target):
            percent_overshoot = (w_max - target) / target * 100
        else:
            percent_overshoot = 0.0
        percent_overshoots.append(percent_overshoot)

    # SS error is based on final value. 
        ss_errs.append(w_i[-1] - target)

    # Rise time: first time to reach 90% target. 
        idx_rise = np.where(w_i >= 0.9* target)[0]
        if len(idx_rise) == 0:
            times_rise.append(np.nan)
        else:
            times_rise.append(t[idx_rise[0]])

    # Settling time: time when the value is within 2% the target speed. 
        idx_settle = np.where(np.abs(w_i - target) <= 0.02 * target)[0]
        if len(idx_settle) == 0:
            times_settle.append(np.nan)
            n_not_settled += 1
        else:
            times_settle.append(t[idx_settle[-1]])
            
    if n_not_settled:
        print(f"Signal did not settle {n_not_settled} times.")

# Convert to np arrays 
    times_rise         = np.array(times_rise)
    times_settle       = np.array(times_settle)
    percent_overshoots = np.array(percent_overshoots)
    ss_errs            = np.array(ss_errs)

    return times_rise, times_settle, percent_overshoots, ss_errs