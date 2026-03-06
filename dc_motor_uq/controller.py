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