
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
