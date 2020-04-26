#! /usr/bin/env python
 
from tf import TransformBroadcaster
import rospy
from rospy import Time 
 
def main():
    rospy.init_node('tfMessage')
    b = TransformBroadcaster()
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 1.0)
    rate = rospy.Rate(1.0) 
    
    x, y = 0.0, 0.0
    k=0
    while not rospy.is_shutdown():
        k+=1
        if k == 20:
            x = 10.0
            y = 0.0
        print(k)
        translation = (x, y, 0.0)
        
        
        b.sendTransform(translation, rotation, Time.now(), '/base_link', '/odom')
        rate.sleep()
    
 
 
if __name__ == '__main__':
    main()