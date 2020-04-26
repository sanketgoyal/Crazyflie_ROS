#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
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

URI = 'radio://0/80/2M/E7E7E7E701'

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

rospy.init_node('laser_scan_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)

num_readings = 100
laser_frequency = 40

count = 0
r = rospy.Rate(1.0)
cflib.crtp.init_drivers(enable_debug_driver=False)
cf = Crazyflie(rw_cache='./cache')
while not rospy.is_shutdown():
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
                with Multiranger(scf) as multiranger:
                    keep_flying=True
                    while keep_flying:
                        current_time = rospy.Time.now()

                        scan = LaserScan()

                        scan.header.stamp = current_time
                        scan.header.frame_id = 'laser_frame'
                        scan.angle_min = -1.57
                        scan.angle_max = 1.57
                        scan.angle_increment = 3.14 / num_readings
                        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
                        scan.range_min = 0.0
                        scan.range_max = 100.0

                        scan.ranges = []
                        scan.intensities = []

                        for i in range(0, num_readings):
                            if multiranger.front == None:
                                continue
                            else:
                                scan.ranges.append(multiranger.front)  # fake data
                                scan.intensities.append(1)  # fake data
                                print("in",multiranger.front)

                        scan_pub.publish(scan)
                        count += 1
                        r.sleep()