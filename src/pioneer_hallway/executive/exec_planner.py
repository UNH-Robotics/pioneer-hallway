#! /usr/bin/env python

import subprocess

cur_x = 1.0
cur_y = 1.0
cur_lin = 0.0
cur_rot = 0.0

p = subprocess.Popen("./planner.sh", shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

# timestamp beginning header
p.stdin.write(str(time.time())
p.stdin.write(str(
