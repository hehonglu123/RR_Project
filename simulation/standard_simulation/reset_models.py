#!/usr/bin/env python3
#reset objects and box location to random
import numpy as np
from RobotRaconteur.Client import *


testbed=RRN.ConnectService('rr+tcp://localhost:6666?service=testbed')   #testbed service
testbed.reset=1		#[perfume,toothpaste,soap,bottle]
