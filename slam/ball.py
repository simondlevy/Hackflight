#!/usr/bin/env python

# written by Lenore Horner, 2009

from visual import *

floor = box(length=4, height=0.5, width=4, color=color.blue)

ball = sphere(pos=(0,4,0), color=color.red)
ball.velocity = vector(0,-1,0)
ball.acceleration = vector(0,9.8,0)

power = 2
constant = .5

dt = 0.01
while 1:
    rate(100)
    ball.pos = ball.pos + ball.velocity*dt
    if ball.y < 1:
        ball.velocity.y = -ball.velocity.y
    else:
		ball.velocity.y = ball.velocity.y + ball.acceleration.y*dt
		ball.acceleration.y = -9.8 - sign(ball.velocity.y) * constant * ball.radius * ball.velocity.y ** power
		
