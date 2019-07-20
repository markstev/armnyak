
import sys
import math
from arm_simulator import ArmSimulator
from controller import Controller, MathController
import logging
import SimpleHTTPServer
import SocketServer
from BaseHTTPServer import BaseHTTPRequestHandler
import motor2
import json
import time
import config
import cgi

#root = logging.getLogger()
#root.setLevel(logging.DEBUG)

simulator = ArmSimulator()
arm_config = config.ArmConfig()
simulator.Configure(config.ArmConfig())
simulator.SetTargetAbsolute(math.pi / 3,  arm_config.r0 + arm_config.r1_camera, arm_config.target_width, 7)


PORT = 8000


controller = MathController(arm_config)
bank = motor2.MotorBankBase()
#bank.wrist_motor.Configure(microsteps=4, max_steps=1600, min_steps=-1600)
#bank.base_motor.Configure(microsteps=2,
#        max_steps=int(bank.base_motor.StepsPerRadian() * math.pi / 2),
#        min_steps=int(-bank.base_motor.StepsPerRadian() * math.pi / 2))
simulator.SendCoordsToMotorBank(bank)
#simulator.Simulate(0.1, 0.2, total_time=1, controller=controller)

server_link = None
class MyRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    pause = True
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
        simulator.Step(0.1)
        if simulator.override_camera is None:
          simulator.ControlStep(0.1, controller)
        json_string = json.dumps(simulator.Positions(controller))
	self.send_response(200)
	self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json_string)
        if (abs(simulator.d_theta) < 0.01 and
            abs(simulator.d_phi) < 0.01 and
            abs(simulator.d_rho) < 0.01):
            simulator.Reset()
	return

    def SendOk(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write("ok")

    def do_POST(self):
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
        data = json.loads(self.data_string)
        logging.info(data)
        target = data[u"target"]
        camera_view = config.CameraView(-target[u"horiz_offset_rad"],
                target[u"width_rad"], target[u"vert_offset_rad"])
        simulator.OverrideCamera(camera_view)
        simulator.ControlStep(0.1, controller)
        self.SendOk()
        logging.info("POST DONE IN: %.02f", time.time() - start)
        #simulator.OverrideCamera(data.


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
