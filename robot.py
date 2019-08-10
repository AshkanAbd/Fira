# coding=utf-8
import roslaunch
import subprocess


class Robot:
    def __init__(self, mode=1):
        setup_launch = []
        if mode == 1:
            setup_launch = ['roslaunch', 'aura', 'setup_fast.launch']
        if mode == 2:
            setup_launch = ['roslaunch', 'aura', 'setup_slow.launch']
        setup_process = subprocess.Popen(setup_launch)

    def start(self):
        auto_move_launch = ['roslaunch', 'aura', 'auto_move.launch']
        auto_move_process = subprocess.Popen(auto_move_launch)
