#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = 'radio://0/80/2M/E7E7E7E703'
x = 0
y = 0 
z = 0
if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def is_close(range):
    MIN_DISTANCE = 0.1  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    global x
    global y
    global z
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()
    # log_conf.data_received_cb.add_callback(position_callback)
    

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache='./cache')
    while not rospy.is_shutdown():
        with SyncCrazyflie(URI, cf=cf) as scf:
            with MotionCommander(scf) as motion_commander:
                with Multiranger(scf) as multiranger:
                    #reset_estimator(scf)
                    start_position_printing(scf)
                    
                    keep_flying = True

                    while keep_flying:
                        print("tadaaaaa-----",x)
                        VELOCITY = 0.5
                        velocity_x = 0.0
                        velocity_y = 0.0

                        if is_close(multiranger.front):
                            velocity_x -= VELOCITY
                        if is_close(multiranger.back):
                            velocity_x += VELOCITY

                        if is_close(multiranger.left):
                            velocity_y -= VELOCITY
                        if is_close(multiranger.right):
                            velocity_y += VELOCITY

                        if is_close(multiranger.up):
                            keep_flying = False
                        hello_str = "hello world %s" % multiranger.front

                        rospy.loginfo(hello_str)
                        #pub.publish(hello_str)
                        motion_commander.start_linear_motion(
                            velocity_x, velocity_y, 0)
                        #print(data['kalman.stateX'])
                        time.sleep(0.1)

                print('Demo terminated!')
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
