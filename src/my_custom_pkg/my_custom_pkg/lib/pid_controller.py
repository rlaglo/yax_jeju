import time


class PIDController:
    """범용 PID 컨트롤러 (Windup 방지 포함)"""

    def __init__(self, kp: float, ki: float, kd: float,
                 setpoint: float = 0.0,
                 integral_max: float = 1.0,
                 integral_min: float = -1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_max = integral_max
        self.integral_min = integral_min

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, current_value: float, dt: float = None) -> float:
        if dt is None or dt <= 0:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
        if dt <= 0:
            return 0.0

        error = self.setpoint - current_value

        # P
        p_term = self.kp * error

        # I (windup 방지)
        self.integral += error * dt
        self.integral = max(self.integral_min,
                            min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # D
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        self.prev_error = error
        return p_term + i_term + d_term

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
