
import sys
import math
from arm_simulator import ArmSimulator
from controller import Controller
import logging
import SimpleHTTPServer
import SocketServer
from BaseHTTPServer import BaseHTTPRequestHandler
import json

#root = logging.getLogger()
#root.setLevel(logging.DEBUG)

simulator = ArmSimulator()
simulator.Configure(30, 10)
simulator.SetTargetAbsolute(math.pi / 3,  35, 5, 7)


PORT = 8000


controller = Controller()
#simulator.Simulate(0.1, 0.2, total_time=1, controller=controller)

class MyRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path in ('/quitquitquit'):
            quit()
        if self.path in ('/debug.html', '/three.js', '/OrbitControls.js'):
            return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)
        if self.path in ('/reset'):
            simulator.Reset()
            return
        if self.path in ('/replay'):
            simulator.Replay()
            return
        simulator.Step(0.1)
        simulator.ControlStep(0.1, controller)
        json_string = json.dumps(simulator.Positions())
	self.send_response(200)
	self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json_string)
        if (abs(simulator.d_theta) < 0.01 and
            abs(simulator.d_phi) < 0.01 and
            abs(simulator.d_rho) < 0.01):
            simulator.Reset()
	return

for port in range(PORT, PORT + 1000):
  try:
    httpd = SocketServer.TCPServer(("", port), MyRequestHandler)
    break
  except:
    pass
print "serving at port", port
httpd.serve_forever()
