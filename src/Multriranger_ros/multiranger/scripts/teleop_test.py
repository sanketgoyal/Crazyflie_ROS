#!/usr/bin/env python

from __future__ import print_function
import threading
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""
moveBindings = {
        'i':(1,0,0,0),
        'o':(0,1,0,0),
        'j':(0,0,1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# def get_data(x,y,z):
#     return x
# def send_value(move):
#     value=get_data(x,y,z)
#     move.send(value)
#     move.close()




settings = termios.tcgetattr(sys.stdin)
x = 0
y = 0
z = 0
th = 0
status = 0
def run():
    try:
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
            print(x)

    except Exception as e:
        print(e)

    finally:
        x=0
        y=0
        z=0
        #send_value()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

thread = threading.Thread(target=run)
thread.start()