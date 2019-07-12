#!/usr/bin/python

from motor import Motor

negative_trigger_pin = 4 # unused
positive_trigger_pin = 5 # unused
enable_pin = 13 # EN pin
temp_pin = 20 # unused. Turned on while moving for temp_pin_threshold steps

stepper_dir_pin = 8
stepper_pulse_pin = 9
forward = True
final_wait = 1000
max_wait = 4000
temp_pin_threshold = 20

motor = Motor()
for i in range(3):
  forward = not forward
  steps = 9999
  motor.Move(stepper_dir_pin=stepper_dir_pin,
      stepper_pulse_pin=stepper_pulse_pin,
      negative_trigger_pin=negative_trigger_pin,
      positive_trigger_pin=positive_trigger_pin,
      done_pin=enable_pin,
      forward=forward,
      steps=steps,
      final_wait=final_wait,
      max_wait=max_wait,
      temp_pin=temp_pin,
      temp_pin_threshold=temp_pin_threshold)

