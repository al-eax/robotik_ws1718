
# Simulator starten:

* `source catkin_ws/devel/setup.bash`
* `roslaunch seat_car_gazebo sim.launch`

# server nachträglich starten:
* `/opt/ros/kinetic/lib/gazebo_ros/gzserver -e ode /home/alex/seat/model_car_3/catkin_ws/src/seat_car_simulator/seat_car_gazebo/worlds/empty.world __name:=gazebo __log:=/home/alex/.ros/log/5aebf68e-bf59-11e7-85c9-94659c46b9c8/gazebo-2.log]`

# Vom Terminal publishen/echon

* `rostopic pub /manual_control/speed std_msgs/Int16 "data: -40"`

* `rostopic echo /odom`

# Package starten

* `rosrun ub2 ub2.py`


# Package erstellen
* `cd ~/catkin_ws/src`
* `catkin_create_pkg PACKAGE_NAME std_msgs rospy`
* `cd ~/catkin_ws`
* `catkin_make`
* `cd ~/catkin_ws/src/PACKAGE_NAME`
* `catkin build`
* `mkdir ~/catkin_ws/src/PACKAGE_NAME/scripts`
* `touch ~/catkin_ws/src/PACKAGE_NAME/scripts/SCRIPT_NAME.py`
* `chmod +x ~/catkin_ws/src/PACKAGE_NAME/scripts/SCRIPT_NAME.py`

# Paket starten
* source bash
* `rosrun PACKAGE_NAME SCRIPT_NAME.py`



# Standart Python Imports
```py
#!/usr/bin/env python

import rospy
from math import *
import signal
import sys

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# allow user to terminate script with CTRL+C
def signal_handler(signal, frame):
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)



if __name__ == '__main__':
    try:
		pass
    except rospy.ROSInterruptException:
        pass


```
