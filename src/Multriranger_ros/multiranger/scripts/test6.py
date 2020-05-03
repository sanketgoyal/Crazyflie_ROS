#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import rospy
from std_msgs.msg import String
import logging
import sys
import time
from rospy import Time 
from tf import TransformBroadcaster
from tf import *
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import math
URI = 'radio://0/80/2M/E7E7E7E700'

if len(sys.argv) > 1:
    URI = sys.argv[1]

x = 0
y = 0 
z = 0
yaw = 0 
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
rospy.init_node('laser_scan_publisher')
scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)
count = 0
num_readings = 2
laser_frequency = 40
r = rospy.Rate(1.0)
cflib.crtp.init_drivers(enable_debug_driver=False)
cf = Crazyflie(rw_cache='./cache')
b = TransformBroadcaster()
translation = (0.0, 0.0, 0.0)
rotation = (0.0, 0.0, 0.0,1.0)
x, y = 0.0, 0.0
k=0
keep_flying=True
s=0
angle =  0 
def position_callback(timestamp, data, logconf):
    global x
    global y
    global z
    global yaw
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    yaw = data['stabilizer.yaw'],
    #print('pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    log_conf.add_variable('stabilizer.yaw', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()
while not rospy.is_shutdown():
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as motion_commander:
                with Multiranger(scf) as multiranger:
                    start_position_printing(scf)
                    while keep_flying:
                        #angle = 0 
                        current_time = rospy.Time.now()
                        scan = LaserScan()
                        scan.header.stamp = current_time
                        scan.header.frame_id = 'laser_frame'
                        scan.angle_min = 0
                        scan.angle_max = 2*math.pi
                        scan.angle_increment = 2*math.pi / num_readings
                        scan.time_increment = (1.0 / laser_frequency) / (num_readings)
                        scan.range_min = 0.0
                        scan.range_max = 100.0

                        scan.ranges = []
                        scan.intensities = []
                        if multiranger.front == None:
                            scan.ranges.append(0) 
                            #scan.intensities.append(0) 
                            continue  
                        else:
                            scan.ranges.append(multiranger.front) 
                            #scan.intensities.append(1)
                            # scan.ranges.append(multiranger.right) 
                            # #scan.intensities.append(1)                            
                            scan.ranges.append(multiranger.back) 
                            # #scan.intensities.append(1)
                            # scan.ranges.append(multiranger.left) 
                            #scan.intensities.append(1)
                        k+=1
                        if k>=5:
                            #s+=0.05
                            #motion_commander.move_distance(0.05,0,0,0.2)
                            motion_commander.turn_left(10)
                            angle+=10
                            time.sleep(0.5)
                        if k == 100:
                            break
                        print(k)
                        if k < 5:
                            translation = (0, 0, 0.0)
                            #b.sendTransform(translation,rotation, Time.now(), '/base_link', '/odom')
                        else:
                            translation = (0, 0, 0.0)
                            print("imu",x)
                            print("movement",s)
                            print("yaw", yaw[0])
                            yaw_temp = yaw[0]
                            # if(yaw[0] < 0):
                            #     yaw_temp = yaw[0] + 360

                            b.sendTransform(translation,transformations.quaternion_from_euler(0, 0, yaw[0]*(math.pi/180)), Time.now(), '/base_link', '/odom')
                            #b.sendTransform(translation,rotation, Time.now(), '/base_link', '/odom')
                            scan_pub.publish(scan)
                        time.sleep(1)
                        count += 1
                        r.sleep()
                    keep_flying=False
                    r.sleep()
