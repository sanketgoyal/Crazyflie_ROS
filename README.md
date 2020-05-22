# Crazyflie_ROS

To peform gmapping using Crazyflie 

```
Terminal 1
./launch.sh

Terminal 2
rosrun multiranger test5.py

Terminal 3
rosrun multiranger test9.py
```

To peform localization and path planning using Crazyflie

```
Terminal 1
roslaunch localize amcl.launch

Terminal 2
rosrun multiranger test5.py

Terminal 3
rosrun multiranger test10.py
```
