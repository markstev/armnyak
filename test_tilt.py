
from motor2 import MotorBankBase
import time

bank = MotorBankBase()
bank.wrist_tilt_motor.Configure(microsteps=1, max_steps=600, min_steps=-600)

forward = 1.0
for i in range(2):
  forward = -1.0 * forward
  pos = forward * 0.5
  speed = 1.0
  bank.wrist_tilt_motor.MoveAbsolute(speed, pos)
  time.sleep(3.0)
