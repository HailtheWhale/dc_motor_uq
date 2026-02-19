import numpy as np
import matplotlib.pyplot as plt
# Solving IVP
from scipy.integrate import solve_ivp
# For Latin hypercube sampling
from scipy.stats import qmc, norm
# Progress bars and speeding up processes. 
from tqdm import tqdm
import concurrent.futures
# Visualization Saving
from pathlib import Path


class CtrlPID:
    def __init__(self, target:float, 
                 gains:list[float,float,float]=[0.5,0.001,0.1]):
   
        self.target_w = target
        self.Kp, self.Ki, self.Kd = gains
        # Accumulation terms for I and D terms 
        self.integral = 0.0
        self.err_prev = 0.0

# PID controller. In this case for w. 
    def sim_step(self, actual:float, dt:float):
    # Err is the target speed - actual
        err = self.target_w-actual
        err_dot = (err-self.err_prev)/ dt
    # P,I,D
        P_out = self.Kp*err
        I_out = self.Ki*self.integral
        D_out = self.Kd*err_dot
    # return value.
        value = P_out+I_out+D_out  # PID application
    # Update values
        self.err_prev = err
        self.integral += err*dt

        return value
        
class MotorDC(CtrlPID):
    def __init__(self, 
                 target_w:float                                           = 10,
                 PID_gains:list[float,float,float]                        = [0.5,0.001,0.1],
                 params_mean:list[float,float,float,float,float,float]    = [1.0, 0.01, 0.05, 0.05, 0.01, 0.001],
                 params_std_dev:list[float,float,float,float,float,float] = [0.1, 0.002,0.005,0.005,0.002,0.0002]):

        super().__init__(target = target_w, gains=PID_gains)
    # Motor params: [R, L, Kb, Kt, J, B]
        self.params_mean = params_mean
        self.params_std_dev = params_std_dev
        self.n_params = len(self.params_mean)

#####################
# Motor Model and Solving Method
#####################
# The DC Motor Model for solve_ivp
    def motor_model(self, t:float, y:np.ndarray, params:np.ndarray):
    # y is [current,w]. Returns this when solved. 
    # Incorporate the PID to the solver fun. 
        Vt = self.sim_step(actual=y[1], dt=self.dt)
    # Motor params: [R, L, Kb, Kt, J, B]
    # Need to be fed here separate because will change depending on distribution
        R, L, Kb, Kt, J, B = params

        didt = (Vt - Kb*y[1] - R*y[0])/L
        dwdt = (Kt*y[0] - B*y[1])/J
        return np.array([didt,dwdt])

# Solving the motor model. 
    def motor_model_solve(self, params_i:np.ndarray, y0:list[float,float], 
                          t_span:tuple[float,float], t_eval:np.ndarray):
    # args is for additional arguments that are NOT y,t. 
    # Order of sols is defined in motor_model. 
    # Each returned val is a vector. 
        current_now, w_now = solve_ivp(fun = self.motor_model, t_span=t_span, y0=y0, args=(params_i,), t_eval=t_eval).y
        return current_now, w_now

#####################
## Sampling Methods 
#####################
    def monte_carlo_sampling(self, n_samples:int=1000):
    # Make matrix to store vals. n_samples x n_params
        self.params = np.zeros((n_samples, self.n_params))
    # Perform sampling for all parameters. 
    # They all follow normal distributions. 
        for i,val in enumerate(self.params_mean):
            self.params[:,i] = np.random.normal(val, self.params_std_dev[i], n_samples).astype(np.float16)

    def lhs_sampling(self, n_samples:int=1000):
    # First, pull in params and organize. Need cols for multiplication.
        self.params_mean = np.array(self.params_mean).reshape(-1,1)
        self.params_std_dev = np.array(self.params_std_dev).reshape(-1,1)

    # Refer: https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.qmc.LatinHypercube.html
    # Makes n points in [0,1]^d. Each value is stratified. 
    # Make hypercubes based on however many samples working, d. 
        sampler = qmc.LatinHypercube(d=n_samples)
    # In each of these, sample for each parameter. 
    # This will give a n x d matrix back. 
        sample = sampler.random(n=self.n_params)

    # Refer: https://stackoverflow.com/questions/20626994/how-to-calculate-the-inverse-of-the-normal-cumulative-distribution-function-in-p
    # norm.ppf finds the inverse of the cumulative distribution function for a normal distribution ("norm").
    # ppf for "percent point function" or "quantile function".
    # By default, uses mean = 0 and stddev = 1. loc and scale change this. 

    # Sample is the quantile. So the n_params x n_samples matrix is finding the quantiles for each sample and param. 
    # Need to transpose it to convert it back to n_samples x n_params format. 
        self.params = norm.ppf(sample, loc=self.params_mean, scale=self.params_std_dev).T.astype(np.float16)
        
    def importance_sampling(self,n_samples:int=1000):
    # Make matrix to store vals. n_samples x n_params
        self.params=np.zeros((n_samples,self.n_params))
    
        for i,val in enumerate(self.params_mean):
        # Based on the 3 sigma rule where 99.7% of values fall within 3 std devs
        # from the mean, want to sample for values within that. 
            bnd_low  = val - 3*self.params_std_dev[i]
            bnd_high = val + 3*self.params_std_dev[i]
            self.params[:,i] = np.random.uniform(low=bnd_low, high=bnd_high, size=n_samples).astype(np.float16)


#####################
## Solving
#####################
    def solver(self, 
                dt:float=0.001, t_span:tuple[float,float]=(0.0,1.0), 
                n_samples:int=1000, sampler:str="MC",
                y0:list[float,float]=[0.0,0.0]):
        self.n_samples = n_samples # for visuals access
        self.dt = dt
        self.t_eval  = np.arange(*t_span, self.dt)
    # Current, motor speed: w
        self.current = []
        self.w       = []

    # Monte Carlo Sampling
        if sampler == "MC":
            self.monte_carlo_sampling(n_samples=self.n_samples)
    # Latin Hypercube Sampling
        elif sampler == "LHS":
            self.lhs_sampling(n_samples=self.n_samples)
        elif sampler == "IS":
            self.importance_sampling(n_samples=self.n_samples)
        else:
            raise ValueError("Only: 'MC', 'LHS', and 'IS' sampling allowed.")

    # Loop through all the samples in tqdm. Will take a while. 
        with tqdm(total=self.n_samples) as progress:
        # Open up multi-threading until complete entire dataset. 
            with concurrent.futures.ProcessPoolExecutor(max_workers=40) as exec:
            # Submit a bunch of different samples in a for loop to different processors for faster handling. 
                futures = {exec.submit(self.motor_model_solve, params_i, y0, t_span, self.t_eval) for params_i in self.params}
                # As these portions are finished, they will be flagged. Update the progress bar when this happens. 
                for future in concurrent.futures.as_completed(futures):
                    progress.update(1)
                    current_now, w_now = future.result()
                    self.current.append(current_now)
                    self.w.append(w_now)
    
    # Convert lists to np arrays for faster handling
    # These are 2D matrices, with x being the sample number. 
        self.current = np.array(self.current)
        self.w       = np.array(self.w)

#####################
## Plotting and Analyzing
#####################

# Mean and uncertainty plotting. 
    def solver_w_plot(self, title:str=None, save_dir:str="Images"):
    # Save the figures. There's a lotta them. 
        save_dir = Path(save_dir)
    # Make sure it exists.
        if not save_dir.exists():
            save_dir.mkdir()

    # Getting the plots for the motor speed, bc PID ctrls this. 
        w_mean     = self.w.mean(0)
        w_std      = self.w.std(0)
    # Envelope of uncertainty.
        w_bnd_low  = w_mean - w_std
        w_bnd_high = w_mean + w_std

    # Plotting the results 
        fig, ax = plt.subplots()
        ax.plot(self.t_eval, w_mean)
    # Probability Envelope. 
        ax.fill_between(self.t_eval, w_bnd_low, w_bnd_high, alpha=0.3)
        ax.set_xlabel("Time (s)")
        # https://pythonforundergradengineers.com/unicode-characters-in-python.html
        ax.set_ylabel("\u03C9 (rad/s)")
        ax.grid()
        ax.set_title(title)
        fig.savefig(save_dir/ f"{title}.png")
        plt.close()

# What the histogram visuals use. 
    def get_performance_metrics(self):
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
                if (w_max > self.target_w):
                    percent_overshoot = (w_max-self.target_w)/self.target_w*100
                else:
                    percent_overshoot = 0.0
                self.percent_overshoots.append(percent_overshoot)
            # SS error is based on final value. 
                self.ss_errs.append(w_i[-1]-self.target_w)
            # Rise time is the first time to reach 90% the target speed. 
                self.times_rise.append(self.t_eval[w_i > self.target_w*0.9][0])
            # Settling time is the time when the value is within 2% the target speed. 
                try:
                    time_settle = self.t_eval[np.where(abs(w_i - self.target_w) <= 0.02*self.target_w)[-1][0]]
                    self.times_settle.append(time_settle)
                except:
                    print("Signal did not settle.")
        # Convert to np arrays 
            self.times_rise         = np.array(self.times_rise).astype(np.float16)
            self.times_settle       = np.array(self.times_settle).astype(np.float16)
            self.percent_overshoots = np.array(self.percent_overshoots).astype(np.float16)
            self.ss_errs            = np.array(self.ss_errs).astype(np.float16)


            return self.times_rise,self.times_settle,self.percent_overshoots,self.ss_errs
    
    def solver_histo_plot(self, save_dir:str="Images", title:str=None):
    # Save the figures. There's a lotta them. 
        histograms_dir = Path(save_dir) / "Histograms"
        if not histograms_dir.exists():
            histograms_dir.mkdir()

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

#####################
## Full Pipeline
#####################
def main():
# Load in the motor parameters
# Params: [R, L, Kb, Kt, J, B]
    params_mean    = [1.0, 0.01, 0.05, 0.05, 0.01, 0.001]
    params_std_dev = [0.1, 0.002,0.005,0.005,0.002,0.0002]
    pid_gains      = [5.0,0.1,1.0]
    methods        = ["MC","LHS", "IS"]

# How many samples should use for each method. The more samples used the longer
# it will take to run and the more memory. Can expand to multiple runs if wanted, 
# but will make even more figs. 
    for i in [250,500, 1000]:
    # Loop over all the different methods.
        for j in methods:
            dc_motor = MotorDC(target_w=10.0,PID_gains=pid_gains,
                            params_mean=params_mean,params_std_dev=params_std_dev)
            print(f"Solving for {j}. {i} Samples being used.")
        # Solves for w and current
            dc_motor.solver(dt=0.0001,t_span=(0.0,1.0),n_samples=i,sampler=j,y0=[0.0,0.0])
        # Plots w response. 
            dc_motor.solver_w_plot(title=f"{j} \u03C9 vs Time for {i} Samples")
        # Gets the performance metricss and makes a histogram out of it. 
            dc_motor.solver_histo_plot(title=f"{j} metric for {i} Samples")
        # Display them for sanity checks
            dc_motor.metric_printer()
            print("\n")
        # Cleanup to prevent memory leaks
            del dc_motor

# Sensitivity analysis. Will changing the std deviations change anything?
    types = ["normal","R", "L", "Kb", "Kt","J","B"]
    for i,type in enumerate(types):
        print(f"Solving for sensitivity with {type}. 100 Samples being used.")
    # For the given type, make uncertainty 0. Look at performance results. 
        if type != "normal":
        # Params: [R, L, Kb, Kt, J, B]
            params_std_dev[i-1] = 0.0
        dc_motor = MotorDC(target_w=10.0,PID_gains=pid_gains,
                        params_mean=params_mean,params_std_dev=params_std_dev)
    # Solves for w and current
        dc_motor.solver(dt=0.0001,t_span=(0.0,1.0),n_samples=100,sampler="MC",y0=[0.0,0.0])
    # Plots w response. 
        dc_motor.solver_w_plot(title=f"Uncertainty {type} MC \u03C9 vs Time for 100 Samples")
    # Check the data. 
        dc_motor.get_performance_metrics()
        dc_motor.metric_printer()
        print("\n")
    
    # Need to reset each time. 
        if type != "normal":
            params_std_dev = [0.1, 0.002,0.005,0.005,0.002,0.0002]

    # Cleanup to prevent memory leaks
        del dc_motor
        
if __name__ == '__main__':
    main()