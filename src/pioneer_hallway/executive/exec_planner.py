#! /usr/bin/env python

import subprocess

p = subprocess.Popen("./planner.sh", shell=True, stdin=subprocess.PIPE)
