class PID():
    def __init__(self):
        self.ref = 1.9
        self.kp = 7
        self.ki = 0.0
        self.kd = 0.2
        self.delta_t = 0.05 # 50 ms
        self.e_acum = 0.0
        self.measu_prev1 = 0.0
        self.measu_prev2 = 0.0
        self.measu_prev3 = 0.0
        self.sat_e_acum = 60.0 # 60 degrees of tilt angle error
    
    def loop(self, measurement):
        filtered_measu = (measurement + self.measu_prev1 + self.measu_prev2 + \
            self.measu_prev3) / 4
        e_k = self.ref - filtered_measu
        u_p = self.kp * e_k
        u_i = self.ki * (self.e_acum + e_k) * self.delta_t
        u_d = self.kd * (self.measu_prev1 - measurement) / self.delta_t
        u = u_p + u_i + u_d
        self.measu_prev3 = self.measu_prev2
        self.measu_prev2 = self.measu_prev1
        self.measu_prev1 = measurement
        self.e_acum += e_k
        if self.e_acum > self.sat_e_acum:
            self.e_acum = self.sat_e_acum
        elif self.e_acum < - self.sat_e_acum:
            self.e_acum = - self.sat_e_acum
        
        return u



