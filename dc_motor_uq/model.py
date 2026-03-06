import numpy as np
from scipy.integrate import solve_ivp
from concurrent.futures import ProcessPoolExecutor
from tqdm import tqdm
   
from .controller import CtrlPID
from .sampling import monte_carlo_sampling, lhs_sampling, uniform_sampling
from .metrics import compute_metrics

# Parallelism. Belongs in the MotorDC.
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

    # Treat V as const. input for each integration window.
    def motor_model(self, t:float, y:np.ndarray, params:np.ndarray, Vt):
        R, L, Kb, Kt, J, B = params
        i, w = y
        # Physical Equations
        didt = (Vt - Kb*w - R*i)/L
        dwdt = (Kt*i - B*w)/J
        return [didt,dwdt]

    def solver(self, 
                dt:float=0.001, t_span:tuple=(0.0,1.0), 
                n_samples:int=1000, sampler:str="MC",
                y0:list=[0.0,0.0]):
        self.n_samples = n_samples # for visuals access
        self.dt = dt
        self.t_eval  = np.arange(*t_span, dt)

        # Adjust params based on sampler used.
        if sampler == "MC":
            params = monte_carlo_sampling(params_mean = self.params_mean,
                                          params_std_dev = self.params_std_dev,
                                          n_samples = n_samples)
        elif sampler == "LHS":
            params = lhs_sampling(params_mean = self.params_mean,
                                  params_std_dev = self.params_std_dev,
                                  n_samples = n_samples)
        elif sampler == "US":
            params = uniform_sampling(params_mean = self.params_mean,
                                      params_std_dev = self.params_std_dev,
                                      n_samples = n_samples)
        else:
            raise ValueError("Only: 'MC', 'LHS', and 'IS' sampling allowed.")

        args_list = [
            (params[i], y0, self.t_eval, self.target_w, [self.Kp, self.Ki, self.Kd])
            for i in range(n_samples)
        ]
        self.params = params
    
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

    def get_performance_metrics(self):
        self.times_rise, self.times_settle, self.percent_overshoots, self.ss_errs = compute_metrics(t = self.t_eval,
                                                                                                    n_samples = self.n_samples,
                                                                                                    w = self.w,
                                                                                                    target = self.target_w)
        
    def metric_printer(self):
        # Ensure metrics exist.
        if not hasattr(self,"percent_overshoots"):
            self.get_performance_metrics()

        print(f"Max motor speed DEVIATION (rad/s): {np.max(self.w.std(axis=0))}")

        mean_ctrl_metrics = {
                            "mean_percent_overshoot":np.nanmean(self.percent_overshoots),
                             "mean_steady_state_error":np.nanmean(self.ss_errs),
                             "mean_settling_time":np.nanmean(self.times_settle),
                             "mean_rise_time":np.nanmean(self.times_rise)
        }
        print(mean_ctrl_metrics)
        std_ctrl_metrics = {
                            "std_percent_overshoot":np.nanstd(self.percent_overshoots),
                             "std_steady_state_error":np.nanstd(self.ss_errs),
                             "std_settling_time":np.nanstd(self.times_settle),
                             "std_rise_time":np.nanstd(self.times_rise)
                             }
        print(std_ctrl_metrics)
