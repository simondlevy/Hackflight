'''
  Play back recorded flight-control simulator demands from .CSV file
 
  Copyright (C) 2024 Simon D. Levy, Tiffany Guo, James Xia
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
'''

from controller import Robot
from mixers import runcf
import numpy as np

def makeMotor(robot, name, direction):

    motor = robot.getDevice(name)

    motor.setPosition(float('+inf'))
    motor.setVelocity(direction)

    return motor

def main():

    arr = np.genfromtxt("demands.csv", delimiter=",")

    # Read motors.csv into a numpy array

    robot = Robot()

    m1 = makeMotor(robot, 'motor1', +1)
    m2 = makeMotor(robot, 'motor2', -1)
    m3 = makeMotor(robot, 'motor3', +1)
    m4 = makeMotor(robot, 'motor4', -1)

    timestep = int(robot.getBasicTimeStep())

    i = 0

    while robot.step(timestep) != -1:

        if i < len(arr):
            demands = arr[i]
            i+= 1

        m1val, m2val, m3val, m4val = runcf(demands)

        m1.setVelocity(+m1val)
        m2.setVelocity(-m2val)
        m3.setVelocity(+m3val)
        m4.setVelocity(-m4val)

main()
