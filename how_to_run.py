# coding=utf-8
from robot import Robot
import sys


def Run(robot):
    robot.start()
    return


if __name__ == '__main__':
    mode = 1
    if len(sys.argv) == 3:
        mode = int(sys.argv[1])
        robot = Robot(mode)
        Run(robot)
    elif len(sys.argv) == 2:
        mode = int(sys.argv[1])
        robot = Robot(mode)
        raw_input("Enter any key to start: ")
        Run(robot)
