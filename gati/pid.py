import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0
        self.previous_time = None

    def start(self):
        self.previous_time = time.time()

    def update(self, setpoint, measured_value):
        current_time = time.time()
        if self.previous_time is None:
            raise Exception("PIDController has not been started. Call start() before update().")
        
        dt = current_time - self.previous_time
        error = setpoint - measured_value

        P = self.kp * error

        self.integral += error * dt
        I = self.ki * self.integral

        derivative = (error - self.previous_error) / dt
        D = self.kd * derivative

        self.previous_time = current_time
        self.previous_error = error

        output = P + I + D

        return output