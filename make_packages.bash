cd catkin_ws
catkin_make
cd src
for folder in */ ; do
	cd $folder;
	catkin build;
	cd .. 
done 
source ../devel/setup.bash

