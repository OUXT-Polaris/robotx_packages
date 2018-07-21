#!/usr/bin/env python
# license removed for brevity
import rospy
import rospkg
#import commands
import subprocess
import sys

def get_lines(cmd):
    proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    while True:
        line = proc.stdout.readline()
        if line:
            yield line

        if not line and proc.poll() is not None:
            break

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    path = rospack.get_path('robotx_docker_simulation') + "/docker_compose"
    cmd = "cd " + path + " && docker-compose build"
    for line in get_lines(cmd):
        sys.stdout.write(line)