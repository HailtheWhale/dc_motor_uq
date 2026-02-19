# DC Motor Uncertainty Simulation Project
## Introduction
Whenever using DC motors, parameters such as armature resistance ($R$), armature inductance ($L$), 
or back-EMF constant ($K_b$) will vary because of manufacturing tolerances and environmental conditions. 
Therefore, it is important to consider the effects of uncertainties during use to determine the motor’s potential 
applications. It can also be worthwhile simulating the motor performance before applying it to ensure it will work 
properly regardless of the scenario. 

This project involved the design of a PID-controlled DC motor simulator that included the effects 
of uncertainty for a step angular speed input, ω. It explored various Monte Carlo Simulator techniques 
including Standard Monte Carlo Sampling (MC), Latin Hypercube Sampling (LHS), and Importance Sampling (IS).  
For performance evaluation, this report compares the $\omega$ response vs time in plots with 1000 samples. 
It also compares rise time, settling time, peak overshoot, and steady state error for the different methods 
at different sample sizes represented as histograms, the majority of which are in Images folder for a sample 
size of 250. In general, LHS responses settled fastest with the least overshoot. The performance between 
the models became more exaggerated and fewer samples were taken. Sensitivity analysis was also performed
on each parameter. It was found that when varying parameters, $J$ had the most significant impact on uncertainty. 

## Background
For this simulator, the DC motor dynamics were given by:

$V_a = Ri_a + L\frac{di_a}{dt}+K_b\omega$

$J\frac{d\omega}{dt}=K_ti_a-B\omega$

Where $V_a$ was the applied armature voltage based on a PID output. 
$i_a$ was the armature current. $\omega$ was the angular speed. 
The rest of the values were system parameters, with values defined in Table 1. 
The PID controller used was a standard PID, with the error term $e(t)$ being based 
on the difference between the desired $\omega$ and actual. 

## Methods
The simulator was implemented using Python3. Within the simulator, for each simulation step the PID controller was called to get a voltage response which was used in scipy’s solve_ivp function. The target speed for all trials was 10 rad/s, with an initial state of 0 rad/s, 0 V, and 0 A. The simulator ran for a span of 1 second each trial. The PID gains used were as follows: P = 5.0, I = 0.1, D = 1.0. Before performing the simulation, N samples were retrieved using the given methods: Standard Monte Carlo Sampling (MC), Latin Hypercube Sampling (LHS), and Importance Sampling (IS).  

For this simulator, each motor parameter followed a normal distribution around a nominal value, with specifics shown in Table 1. This was applied for both the MC and LHS methods, but not for IS in which a different distribution was defined to try and improve performance. For IS, uniform samples between 3 standard deviations of the values shown in Table 1 were taken. 

<p align="center">
  <img src="Images_readme/Table 1.png" width="450">
</p>

For performance comparisons, ω vs time for the 3 implementations were plotted. Shown are the plots for 250 samples, in which the performance difference is most noticeable. Performance metrics for rise time, settling time, peak overshoot, and steady state error for the different methods at different sample sizes were represented as histograms, which for 250 samples are in the Images folder. An example of one is provided to give the general feel, but for quantitative analysis, the mean and standard deviation of each trial was tabulated. Sensitivity analysis was then performed on each parameter by zeroing the standard deviation for that parameter for a MC simulation with 100 samples.


