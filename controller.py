import math

class MotorController(object):
    def __init__(self, m, b):
        self.m = m
        self.b = b
        self.history = []

    def GetNextSpeed(self, target_offset, time_delta, other_contributor):
        """
        Returns an angular speed to drive a motor at.

        target_offset and output speed are basically unknown units. Let's see if we can figure them out on the fly.
        
        Assume we're outputting rads/sec
        """
        output = (target_offset - self.b) / self.m
        chosen_output = max(-1.0, min(1.0, output))
        self.history.append((target_offset, time_delta, other_contributor, chosen_output))
        # TODO: regressions
        return chosen_output


class Controller(object):
    def __init__(self):
        self.base_motor = MotorController(0.2, 0)
        self.lift_motor = MotorController(0.1, 0)
        self.wrist_motor = MotorController(0.4, 0)

    def Update(self, offset_tuple, time_delta):
        """
        Base target is distance to target, inferred from width_radians
        Lift target is vertical_offset_radians -> 0
        Wrist target is horiz_offset_radians -> 0
        """
        rho = self.lift_motor.GetNextSpeed(offset_tuple[2], time_delta, 0)
        target_width_radians = 0.7  # 5cm width at 10cm away
        theta = self.base_motor.GetNextSpeed(target_width_radians - offset_tuple[1], time_delta, -rho)
        phi = self.wrist_motor.GetNextSpeed(offset_tuple[0], time_delta, theta)
        return (theta, phi, rho)
