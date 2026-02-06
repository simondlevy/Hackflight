# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This controller gives to its robot the following behavior:
According to the messages it receives, the robot change its
behavior.
"""

from controller import Robot, GPS

class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)


class Slave(Robot):

    Mode = Enumerate('STOP MOVE_FORWARD AVOIDOBSTACLES TURN')
    timeStep = 32
    maxSpeed = 10.0
    motors = []
    distanceSensors = []

    def boundSpeed(self, speed):
        return max(-self.maxSpeed, min(self.maxSpeed, speed))

    def __init__(self):

        self.logfile = open("log.csv", "w")

        super(Slave, self).__init__()

        self.motors.append(self.getDevice("left wheel motor"))
        self.motors.append(self.getDevice("right wheel motor"))
        self.motors[0].setPosition(float("inf"))
        self.motors[1].setPosition(float("inf"))
        self.motors[0].setVelocity(0.0)
        self.motors[1].setVelocity(0.0)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)

        for dsnumber in range(0, 2):
            self.distanceSensors.append(self.getDevice('ds' + str(dsnumber)))
            self.distanceSensors[-1].enable(self.timeStep)

    def run(self):

        while True:

            loc = self.gps.getValues()
            self.logfile.write("%f,%f,%f\n" % (loc[0], loc[1], loc[2]))
            self.logfile.flush()

            delta = self.distanceSensors[0].getValue() - self.distanceSensors[1].getValue()
            speeds = [0.0, 0.0]

            speeds[0] = self.boundSpeed(self.maxSpeed / 2 + 0.1 * delta)
            speeds[1] = self.boundSpeed(self.maxSpeed / 2 - 0.1 * delta)

            self.motors[0].setVelocity(speeds[0])
            self.motors[1].setVelocity(speeds[1])

            if self.step(self.timeStep) == -1:
                break

        self.logfile.close()


controller = Slave()
controller.run()
