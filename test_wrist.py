from motor2 import MotorBankBase
import time

bank= MotorBankBase()
bank.left_grip.SetDisableAfterMoving(False)
bank.right_grip.SetDisableAfterMoving(False)
bank.left_grip.Configure(microsteps=1, max_steps=600, min_steps=-600)
bank.right_grip.Configure(microsteps=1, max_steps=600, min_steps=-600)
bank.wrist_tilt_motor.Configure(microsteps=1, max_steps=600, min_steps=-600)

forward = 1.0
for i in range(1):
  forward = -1.0 * forward
  pos = forward * 0.3
  speed = 0.7
  bank.left_grip.MoveAbsolute(speed, pos)
  bank.right_grip.MoveAbsolute(speed, -1.0 * pos)
  time.sleep(3.0)
  #if i % 2 == 0:
  #or direction in [-1, 1]:
  #   bank.right_grip.MoveAbsolute(speed, -1.0 * pos + direction * .8)
  #   bank.left_grip.MoveAbsolute(speed, pos + direction * .8)
  #   time.sleep(3.0)

bank.left_grip.SetDisableAfterMoving(True)
bank.right_grip.SetDisableAfterMoving(True)
bank.left_grip.MoveAbsolute(0.4, 0.0)
bank.right_grip.MoveAbsolute(0.4, 0.0)
