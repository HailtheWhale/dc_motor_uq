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
For performance evaluation, this README compares the $\omega$ response vs time in plots with 1000 samples. 
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

For performance comparisons, $\omega$ vs time for the 3 implementations were plotted. Shown are the plots for 250 samples, in which the performance difference is most noticeable. Performance metrics for rise time, settling time, peak overshoot, and steady state error for the different methods at different sample sizes were represented as histograms, which for 250 samples are in the Images folder. An example of one is provided to give the general feel, but for quantitative analysis, the mean and standard deviation of each trial was tabulated. Sensitivity analysis was then performed on each parameter by zeroing the standard deviation for that parameter for a MC simulation with 100 samples.

## Results
Figures 1-3 are the ω responses vs time for each method when run for 250 samples. They are too alike to evaluate the mean performance differences, so this is done later when talking about performance metrics. For the uncertainty envelope, MC performed better than LHS and IS. LHS seemed to have a larger uncertainty bound above the curve compared to MC. IS performed worse than the other two, with a larger envelope overall. 

<p align="center">
  <img src="Images_readme/Figure 1.png" width="450">
  <br>
  <em>Figure 1: Angular speed vs time PID step response for 250 MC samples.</em>
</p>

<p align="center">
  <img src="Images_readme/Figure 2.png" width="450">
  <br>
  <em>Figure 2: Angular speed vs time PID step response for 250 LHS samples.</em>
</p>

<p align="center">
  <img src="Images_readme/Figure 3.png" width="450">
  <br>
  <em>Figure 3: Angular speed vs time PID step response for 250 IS samples.</em>
</p>

Figure 4 is an example histogram that shows one of the performance metrics evaluated for each of the cases in this simulator. In this case, it is for MC and is roughly gaussian, which reflects the distribution used in the simulator. The Images folder has the rest of these for 250, 500, and 1000 samples respectively. 

<p align="center">
  <img src="Images_readme/Figure 4.png" width="450">
  <br>
  <em>Figure 4: Histogram describing distribution of settling time for MC method with 250 samples taken.</em>
</p>

For brevity, analysis was based on the mean value and standard deviation for the performance metrics, as shown in Table 2. At large sample counts, the performance metrics between the 3 methods didn’t deviate much for SS Err, settling time, or rise time. The percentage overshoot for the IS method though was much higher. As the sample count is reduced, LHS, begins performing better than the other methods, having increasingly better settling time, rise time, and percent overshoot. 

The mean steady state error for all the methods regardless of the sample size remained consistent. This is because it is based on the tuning of the PID gains themselves. The standard deviation between MC and LHS remained consistent and much lower than the IS. For 250 samples, the settling time was best for LHS with 0.426 seconds, followed by MC with 0.434 and IS with 0.440. The rise time for all methods remained consistent regardless of the sample size. In general, LHS outperformed the other methods in both accuracy and efficiency and converged the fastest. IS did not reduce computational cost while maintaining good accuracy, likely because the chosen distribution for the sampling was not ideal. 

<p align="center">
  <img src="Images_readme/Table 2.png" width="450">
</p>

Table 3 describes the max standard deviation for each method for different sample sizes. IS had consistently worse deviation than MC and LHS. Reducing the sample size seemed to reduce the standard deviation for MC and LHS, but not IS. Interestingly, the max standard deviation for MC was lower than LHS despite the average standard deviation being lower as shown in Table 2. 

<p align="center">
  <img src="Images_readme/Table 3.png" width="450">
</p>

Table 4 describes the sensitivity analysis performed. For this, the MC method was used with 100 samples. In each trial, one of the 6 parameter’s standard deviation was dropped to 0, and the simulation was run. The difference in max standard deviation between that trial and a baseline was then determined. Doing this for most parameters did not significantly affect the max deviation, but for J, the difference was 0.34 deviations. Thus, this system is sensitive to changes in J

<p align="center">
  <img src="Images_readme/Table 4.png" width="450">
</p>

## Conclusions
In general, LHS outperformed the other methods in both accuracy and efficiency and converged the fastest. IS did not reduce computational cost while maintaining good accuracy, likely because the chosen distribution for the sampling was not ideal. IS had consistently worse max standard deviation than MC and LHS. Reducing the sample size seemed to reduce the standard deviation for MC and LHS, but not IS, again likely due to less-than-ideal sampling choice. This system was sensitive to changes in the $J$ parameter’s uncertainty, but not the others. This highlights the importance of checking for this and keeping the uncertainties for sensitive parameters under control to have a trustworthy system. 

