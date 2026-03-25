# src/decision_making_pkg/decision_making_pkg/lib/pid_controller.py

import time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, current_value, dt):
        #current_time = time.time()
        #dt = current_time - self.last_time
        
        if dt <= 0:
            return 0.0

        error = self.setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Compute the control output
        output = p_term + i_term + d_term
        
        # Update state for next iteration
        self.prev_error = error
        
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()