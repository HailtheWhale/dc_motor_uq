import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
# For Latin hypercube sampling
from scipy.stats import qmc, norm
# Progress bars and speeding up processes. 
from concurrent.futures import ProcessPoolExecutor
from tqdm import tqdm

# Pathing
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parent

class CtrlPID:
    def __init__(self, target:float, 
                 gains:list=[0.5,0.001,0.1]):
   
        self.target_w = target
        self.Kp, self.Ki, self.Kd = gains
        self.integral = 0.0
        self.err_prev = 0.0

# PID controller. In this case for w. 
    def sim_step(self, actual:float, dt:float):
        err = self.target_w-actual
        err_dot = (err-self.err_prev)/ dt
        P = self.Kp*err
        I = self.Ki*self.integral
        D = self.Kd*err_dot
        self.integral += err*dt
        self.err_prev = err
        return P + I + D

# Parallelism
def run_single_sample(args):
    params_i, y0, t_eval, target_w, pid_gains = args

    # Fresh PID
    pid = CtrlPID(target_w,pid_gains)

    # Local motor model
    # Treat V as const. input for each integration window.
    def motor_model(t:float, y:np.ndarray, params:np.ndarray, Vt):
        R, L, Kb, Kt, J, B = params
        i, w = y
        # Physical Equations
        didt = (Vt - Kb*w - R*i)/L
        dwdt = (Kt*i - B*w)/J
        return [didt,dwdt]
    
    # Manually time-stepping loop.
    i_now = np.zeros_like(t_eval)
    w_now = np.zeros_like(t_eval)

    # Set IC
    y = np.array(y0)
    i_now[0], w_now[0] = y0

    for k in range(len(t_eval)-1):
        dt = t_eval[k+1] - t_eval[k]

        # Update PID voltage  once per timestep
        Vt = pid.sim_step(actual=w_now[k], dt=dt)

        # Inegrate physics with constant Vt
        sol = solve_ivp(fun = lambda t,y: motor_model(t, y, params_i, Vt), 
                        t_span = (t_eval[k], t_eval[k+1]), 
                        y0 = y, 
                        t_eval = [t_eval[k+1]]
                        )
        
        y = sol.y[:, -1]
        i_now[k+1], w_now[k+1] = y

    return i_now, w_now

class MotorDC():
    def __init__(self, 
                 target_w:float      = 10,
                 PID_gains:list      = [0.5,0.001,0.1],
                 params_mean:list    = [1.0, 0.01, 0.05, 0.05, 0.01, 0.001],
                 params_std_dev:list = [0.1, 0.002,0.005,0.005,0.002,0.0002]):

    # PID Controller instance params
        self.target_w = target_w
        self.Kp, self.Ki, self.Kd = PID_gains

    # Motor params: [R, L, Kb, Kt, J, B]
        self.params_mean = params_mean
        self.params_std_dev = params_std_dev
        self.n_params = len(self.params_mean)

    # Treat V as const. input for each integration window.
    def motor_model(self, t:float, y:np.ndarray, params:np.ndarray, Vt):
        R, L, Kb, Kt, J, B = params
        i, w = y
        # Physical Equations
        didt = (Vt - Kb*w - R*i)/L
        dwdt = (Kt*i - B*w)/J
        return [didt,dwdt]

    def motor_model_solve(self, params_i:np.ndarray, y0:list, 
                          t_span:tuple, t_eval:np.ndarray):
        # fresh PID per sample solved.
        pid = CtrlPID(target=self.target_w, gains=[self.Kp, self.Ki, self.Kd])

        i_now = np.zeros_like(t_eval)
        w_now = np.zeros_like(t_eval)

        # Set IC
        i_now[0], w_now[0] = y0
        y = np.array(y0)

        for k in range(len(t_eval)-1):
            dt = t_eval[k+1] - t_eval[k]

            # Update PID voltage  once per timestep
            Vt = pid.sim_step(actual=w_now[k], dt=dt)

            # Inegrate physics with constant Vt
            sol = solve_ivp(fun = lambda t,y: self.motor_model(t, y, params_i, Vt), 
                            t_span = (t_eval[k], t_eval[k+1]), 
                            y0= y, 
                            t_eval=[t_eval[k+1]]
                            )
            
            y = sol.y[:, -1]
            i_now[k+1], w_now[k+1] = y

        return i_now, w_now

# Sampling Methods
    def monte_carlo_sampling(self, n_samples:int=1000):
    # Make matrix to store vals. n_samples x n_params
        params = np.zeros((n_samples, self.n_params))
    # All following normal distributions. 
        for i,val in enumerate(self.params_mean):
            params[:,i] = np.random.normal(val, self.params_std_dev[i], n_samples)
        return params

    def lhs_sampling(self, n_samples:int=1000):
        means = np.array(self.params_mean)
        std = np.array(self.params_std_dev)

        sampler = qmc.LatinHypercube(d=self.n_params)
        unit_samples = sampler.random(n=n_samples) # shape: (n_samples, n_params)

        params = norm.ppf(unit_samples, loc=means, scale=std)
        return params
        
    def uniform_sampling(self,n_samples:int=1000):
    # Make matrix to store vals. n_samples x n_params
        params=np.zeros((n_samples,self.n_params))
    
        for i,val in enumerate(self.params_mean):
        # Based on the 3 sigma rule where 99.7% of values fall within 3 std devs
        # from the mean, want to sample for values within that. 
            bnd_low  = val - 3*self.params_std_dev[i]
            bnd_high = val + 3*self.params_std_dev[i]
            params[:,i] = np.random.uniform(low=bnd_low, high=bnd_high, size=n_samples)
        return params

# Solving
    def solver(self, 
                dt:float=0.001, t_span:tuple=(0.0,1.0), 
                n_samples:int=1000, sampler:str="MC",
                y0:list=[0.0,0.0]):
        self.n_samples = n_samples # for visuals access
        self.dt = dt
        self.t_eval  = np.arange(*t_span, self.dt)

        # Adjust params based on sampler used.
        if sampler == "MC":
            params = self.monte_carlo_sampling(n_samples=n_samples)
        elif sampler == "LHS":
            params = self.lhs_sampling(n_samples=n_samples)
        elif sampler == "US":
            params = self.uniform_sampling(n_samples=n_samples)
        else:
            raise ValueError("Only: 'MC', 'LHS', and 'IS' sampling allowed.")

    # Loop through all the samples in tqdm. Will take a while. 
        args_list = [
            (params[i], y0, self.t_eval, self.target_w, [self.Kp, self.Ki, self.Kd])
            for i in range(n_samples)
        ]
    
    # Convert lists to np arrays for faster handling
    # These are 2D matrices, with x being the sample number. 
        current = []
        w       = []

        # To ensure deterministic ordering
        with ProcessPoolExecutor() as executor:
            for current_now, w_now in tqdm(executor.map(run_single_sample, args_list), total=n_samples):
                current.append(current_now)
                w.append(w_now)

        self.current = np.array(current)
        self.w = np.array(w)

# Mean and uncertainty plotting. 
    def solver_w_plot(self, title:str=None, save_dir:str="Images"):
        save_dir = BASE_DIR / save_dir
        save_dir.mkdir(exist_ok=True)

    # Getting the plots for the motor speed, bc PID ctrls this. 
    # Ensures NaNs do not break plots. 
        w_mean     = np.nanmean(self.w, axis=0)
        w_std      = np.nanstd(self.w, axis=0)
    # Envelope of uncertainty, 90% credible band.
        w_bnd_low  = np.percentile(self.w, 5, axis=0)
        w_bnd_high = np.percentile(self.w, 95, axis=0)

    # Plotting
        fig, ax = plt.subplots()
        # Probability Envelope. 
        ax.plot(self.t_eval, w_mean, label="Mean Response", color="blue")
        ax.fill_between(self.t_eval, w_bnd_low, w_bnd_high, color="orange", alpha=0.3, label="90% Envelope")
        # Deterministic Trajectory
        det_params = np.median(self.params_mean, axis=0)
        det_i, det_w = self.motor_model_solve(det_params, [0,0], (0, self.t_eval[-1]), self.t_eval)
        ax.plot(self.t_eval, det_w, label="Nominal", color="black", linestyle="--")
        ax.legend()
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("\u03C9 (rad/s)")
        ax.grid()
        ax.set_title(f"{title}\nMean Response with 90% Uncertainty Envelope")
        fig.savefig(save_dir/ f"{title}.png", dpi=300,bbox_inches="tight")
        plt.close(fig)

# What the histogram visuals use. 
    def get_performance_metrics(self):
            target_w = self.target_w

        # Getting Performance Metrics. Each sample taken will have these associated 
        # With them. 
            self.times_rise         = []
            self.times_settle       = []
            self.percent_overshoots = []
            self.ss_errs            = []

            for i in range(self.n_samples):
                w_i   = self.w[i,:] # Pull the RPMs for a given sample
                w_max = max(w_i)    # Find the max value
            # If the max val is more than the target, we have overshoot. 
                if (w_max > target_w):
                    percent_overshoot = (w_max - target_w)/target_w*100
                else:
                    percent_overshoot = 0.0
                self.percent_overshoots.append(percent_overshoot)
            # SS error is based on final value. 
                self.ss_errs.append(w_i[-1]-target_w)
            # Rise time is the first time to reach 90% the target speed. 
                self.times_rise.append(self.t_eval[w_i > target_w*0.9][0])
            # Settling time is the time when the value is within 2% the target speed. 
                try:
                    time_settle = self.t_eval[np.where(abs(w_i - target_w) <= 0.02*target_w)[-1][0]]
                    self.times_settle.append(time_settle)
                except:
                    print("Signal did not settle.")
        # Convert to np arrays 
            self.times_rise         = np.array(self.times_rise)
            self.times_settle       = np.array(self.times_settle)
            self.percent_overshoots = np.array(self.percent_overshoots)
            self.ss_errs            = np.array(self.ss_errs)


            return self.times_rise,self.times_settle,self.percent_overshoots,self.ss_errs
    
    def solver_histo_plot(self, save_dir:str="Images", title:str=None):
        save_dir = BASE_DIR / save_dir
        save_dir.mkdir(exist_ok=True)

    # Retrieve the performance metrics analyzing
        # times_rise,times_settle,percent_overshoots,ss_errs 
        metrics = self.get_performance_metrics()
        x_labels = ["Rise Time (s)","Settling Time (s)",
                    "Percent Overshoot (s)","Steady State Error (rps)"]
        ylabel = "Appearances"

    # Lots of plotting 
        for i,label in enumerate(x_labels):
            fig, ax = plt.subplots()
            ax.hist(metrics[i])
            ax.set_xlabel(label)
            ax.set_ylabel(ylabel=ylabel)
            ax.set_title(title)
            fig.savefig(histograms_dir / f"{label} {title}.png")
            plt.close()

    def metric_printer(self):
        print(f"Max motor speed DEVIATION (RPM): {max(self.w.std(0))}")

        mean_ctrl_metrics = {"mean_percent_overshoot":self.percent_overshoots.mean(),
                             "mean_steady_state_error":self.ss_errs.mean(),
                             "mean_settling_time":self.times_settle.mean(),
                             "mean_rise_time":self.times_rise.mean()}
        print(mean_ctrl_metrics)
        print("\n")

        std_ctrl_metrics = {"std_percent_overshoot":self.percent_overshoots.std(),
                             "std_steady_state_error":self.ss_errs.std(),
                             "std_settling_time":self.times_settle.std(),
                             "std_rise_time":self.times_rise.std()}
        print(std_ctrl_metrics)


def main():
# Load in the motor parameters
# Params: [R, L, Kb, Kt, J, B]
    params_mean    = [1.0, 0.01, 0.05, 0.05, 0.01, 0.001]
    params_std_dev = [0.1, 0.002,0.005,0.005,0.002,0.0002]
    pid_gains      = [5.0,0.1,1.0]
    methods        = ["MC","LHS", "US"]

# How many samples should use for each method. The more samples used the longer
# it will take to run and the more memory. Can expand to multiple runs if wanted, 
# but will make even more figs. 
    i = 250
    j = "MC"
    # for i in [250,500, 1000]:
    # Loop over all the different methods.
        # for j in methods:
    dc_motor = MotorDC(target_w=10.0,PID_gains=pid_gains,
                    params_mean=params_mean,params_std_dev=params_std_dev)
    print(f"Solving for {j}. {i} Samples being used.")
# Solves for w and current
    dc_motor.solver(dt=0.0001, t_span=(0.0,1.0), n_samples=i, 
                    sampler=j, y0=[0.0,0.0])
# Plots w response. 
    dc_motor.solver_w_plot(title=f"{j} \u03C9 vs Time for {i} Samples")
# Gets the performance metricss and makes a histogram out of it. 
    # dc_motor.solver_histo_plot(title=f"{j} metric for {i} Samples")
# Display them for sanity checks
    # dc_motor.metric_printer()
    # print("\n")
# Cleanup to prevent memory leaks
    del dc_motor

# Sensitivity analysis. Will changing the std deviations change anything?
    # types = ["normal","R", "L", "Kb", "Kt","J","B"]
    # for i,type in enumerate(types):
    #     print(f"Solving for sensitivity with {type}. 100 Samples being used.")
    # # For the given type, make uncertainty 0. Look at performance results. 
    #     if type != "normal":
    #     # Params: [R, L, Kb, Kt, J, B]
    #         params_std_dev[i-1] = 0.0
    #     dc_motor = MotorDC(target_w=10.0,PID_gains=pid_gains,
    #                     params_mean=params_mean,params_std_dev=params_std_dev)
    # # Solves for w and current
    #     dc_motor.solver(dt=0.0001,t_span=(0.0,1.0),n_samples=100,sampler="MC",y0=[0.0,0.0])
    # # Plots w response. 
    #     dc_motor.solver_w_plot(title=f"Uncertainty {type} MC \u03C9 vs Time for 100 Samples")
    # # Check the data. 
    #     dc_motor.get_performance_metrics()
    #     dc_motor.metric_printer()
    #     print("\n")
    
    # # Need to reset each time. 
    #     if type != "normal":
    #         params_std_dev = [0.1, 0.002,0.005,0.005,0.002,0.0002]

    # # Cleanup to prevent memory leaks
    #     del dc_motor
        
if __name__ == '__main__':
    main()