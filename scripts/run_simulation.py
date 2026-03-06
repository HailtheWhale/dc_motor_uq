from dc_motor_uq import MotorDC, solver_w_plot, solver_histo_plot

def main():
# Load in the motor parameters
# Params: [R, L, Kb, Kt, J, B]
    params_mean    = [1.0, 0.01, 0.05, 0.05, 0.01, 0.001]
    params_std_dev = [0.1, 0.002,0.005,0.005,0.002,0.0002]
    pid_gains      = [5.0, 0.1, 1.0]

# How many samples should use for each method. The more samples used the longer
# it will take to run and the more memory. Can expand to multiple runs if wanted, 
# but will make even more figs. 
    i = 250
    j = "US"
    # ["MC","LHS", "US"]
    motor = MotorDC(
                    target_w=10.0, 
                    PID_gains=pid_gains,
                    params_mean=params_mean, 
                    params_std_dev=params_std_dev
                    )
    
    print(f"Solving for {j}. {i} Samples being used.")

# Solves for w and current
    motor.solver(
                dt=0.0001, 
                t_span=(0.0,1.0), 
                n_samples=i, 
                sampler=j, 
                y0=[0.0,0.0]
                )
    
    motor.get_performance_metrics()
    motor.metric_printer()

    # Visuals
    solver_w_plot(motor, title=f"{j} \u03C9 vs Time for {i} Samples")
    solver_histo_plot(motor, title=f"{j} metric for {i} Samples")

# Cleanup to prevent memory leaks
    del motor
        
if __name__ == '__main__':
    main()