#!/usr/bin/python
import sys
import math
import logging
import SimpleHTTPServer
import SocketServer
from BaseHTTPServer import BaseHTTPRequestHandler
import motor2
import json
import time
import config
import cgi


from motor2 import MotorBankBase
from io_read import InputBoard
from config import ArmConfig
from protoc.motor_command_pb2 import MotorMoveProto
from hand import Hand
from arm import Arm
import time
import logging
import math

bank = MotorBankBase()
io_reader = InputBoard()
io_reader.Start()
arm_config = ArmConfig()
arm_config.ApplyTares(bank, io_reader)
PORT = 8000

time.sleep(5.0)

hand = Hand(bank, io_reader, arm_config)
arm = Arm(bank, io_reader, arm_config)

server_link = None
class MyRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    pause = True
    track = False
    def do_GET(self):
        if self.path in ('/quitquitquit'):
            server_link.server_close()
        if self.path in ('/debug.html', '/three.js', '/OrbitControls.js'):
            self.pause = True
            return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)
        if self.path in ('/reset'):
            simulator.Reset()
            return self.SendOk()
        if self.path in ('/replay'):
            simulator.Replay()
            return self.SendOk()
        if self.path in ('/toggle_physical'):
            simulator.TogglePhysicalControl()
            return self.SendOk()
        if self.path in ('/healthz'):
            return self.SendOk()
        if self.path in ('/disable_after_move'):
            simulator.SetDisableAfterMove(True)
            return self.SendOk()
        if self.path in ('/enable_after_move'):
            simulator.SetDisableAfterMove(False)
            return self.SendOk()
        if self.path in ('/pause'):
            self.pause = False
            return self.SendOk()
        #   simulator.Step(0.1)
        #   if simulator.override_camera is None:
        #     simulator.ControlStep(0.1, controller)
        #json_string = json.dumps(simulator.Positions(controller))
	self.send_response(200)
	self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write("")
        #   if (abs(simulator.d_theta) < 0.01 and
        #       abs(simulator.d_phi) < 0.01 and
        #       abs(simulator.d_rho) < 0.01):
        #       simulator.Reset()

    def SendOk(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write("ok")

    def do_POST(self):
        logging.info("Handling: %s", self.path)
        if self.path in ('/calibrate_all'):
            self.CalibrateAll()
            return self.SendOk()
        if self.path in ('/go_to_position'):
            self.GoToPosition()
            return self.SendOk()
        if self.path in ('/lower_onto_bottle'):
            self.LowerOntoBottle()
            return self.SendOk()
        if self.path in ('/grab'):
            self.Grab()
            return self.SendOk()
        if self.path in ('/raise_and_dispense'):
            self.RaiseAndDispense()
            return self.SendOk()
        if self.path in ('/reset'):
            self.Reset()
            return self.SendOk()
        if self.path in ('/rezero'):
            self.Rezero()
            return self.SendOk()
        if self.path in ('/toggle_tracker'):
            self.track = not self.track
            return self.SendOk()
        if self.path in ('/full_pickup_routine'):
            self.LowerOntoBottle()
            self.Grab()
            self.RaiseAndDispense()
            self.Reset()
            return self.SendOk()
        if self.path in ('/run_robot'):
            pass
        if self.path in ('/debug.html'):
            form = cgi.FieldStorage(
                fp=self.rfile,
                headers=self.headers,
                environ={'REQUEST_METHOD': 'POST'}
                )
            for name, motor in bank.named_motors.iteritems():
                form_value = form.getvalue(name)
                if form_value:
                    try:
                        logging.info("Moving %s", name)
                        position = float(form_value)
                        motor.MoveAbsolute(0.6, position)
                    except ValueError:
                        logging.info("Can't move to %s", form_value)
            return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)
        start = time.time()
        self.data_string = self.rfile.read(int(self.headers['Content-Length']))
        #Example:
        #{"debug":
        #        {"right": 673, "frame_height_px": 540, "bottom": 584, "top": -185, "frame_width_px": 960, "bounding_box_px": [[282, -108], [262, 563], [673, 584], [637, -185]], "left": 262}, "target": {"horiz_offset_rad": -0.05099531250000007, "width_rad": 1.270246875, "vert_offset_rad": -0.17462031249999999, "height_rad": 2.3766906249999997}}
        if not data_string:
            return
        try:
            data = json.loads(self.data_string)
        except:
            return
        if not data:
            return
        logging.info(data)
        target = data[u"target"]
        if u'target' not in data:
            return
        camera_view = config.CameraView(-target[u"horiz_offset_rad"],
                target[u"width_rad"], target[u"vert_offset_rad"])
        # TODO:: MOVE LEFT/RIGHT ON THETA
        #simulator.OverrideCamera(camera_view)
        #simulator.ControlStep(0.1, controller)
        self.SendOk()
        logging.info("POST DONE IN: %.02f", time.time() - start)
        #simulator.OverrideCamera(data.

    def CalibrateAll(self):
        hand.Calibrate()
        #bank.WaitDone()
        arm.Calibrate()
        bank.WaitDone()

    def GoToPosition(self):
        arm.GoToPositionHeight()
        bank.WaitDone()
    def LowerOntoBottle(self):
        arm.LowerOntoBottle()
        bank.WaitDone()

    def Grab(self):
        hand.Grab()
        bank.WaitDone()

    def RaiseAndDispense(self):
        arm.RaiseFully()
        bank.WaitDone()
        hand.Dispense()
        arm.SetWristToGrabPosition()

    def Reset(self):
        arm.Lower()
        bank.WaitDone()
        hand.Release()
        bank.WaitDone()
        arm.RaiseOverDroppedBottle()
        bank.WaitDone()
        arm.RaiseFully()
        bank.Rezero()
        bank.WaitDone()

    def Rezero(self):
        bank.Rezero()
        bank.WaitDone()


for port in range(PORT, PORT + 1000):
  try:
    httpd = SocketServer.TCPServer(("", port), MyRequestHandler)
    break
  except:
    time.sleep(1)
    pass
server_link = httpd



print "serving at port", port
try: 
  httpd.serve_forever()
except KeyboardInterrupt:
  pass
httpd.server_close()

#   while True:
#       s = "\n------\n"
#       for i in range(22, 32, 2):
#           s += " PIN %d = %d\n" % (i, io_reader.GetPin(i))

#       logging.info("RPS = %f %s", io_reader.ReadsPerSec(), s)
#       time.sleep(0.2)
#hand.Dispense()
#arm.RaiseFully()
#bank.WaitDone()
time.sleep(10.0)
#   #   #arm.Lower()
#   #   #arm.Rotate()
#   #   #   #hand.Hold()
#   #   time.sleep(2.0)
#   #   #   time.sleep(1.0)
#   #   bank.WaitDone()

#   #   arm.SetupLowerStop()
#   #   bank.Rezero()
#   #   #   #hand.Hold()
