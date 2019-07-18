# Units are cm, radians, and seconds

class ArmConfig(object):
    def __init__(self):
        self.r0 = 12 * 2.54
        self.r1 = 13 * 2.54

class ArmCurrentSettings(object):
    def __init__(self):
        self.theta = 0.0
        self.phi = 0.0
        self.rho = 0.0
