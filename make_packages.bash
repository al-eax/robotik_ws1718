cd catkin_ws
catkin_make
cd src
for folder in */ ; do
	cd $folder;
	catkin build;
	cd .. 
done 
cd ..
catkin_make
source devel/setup.bash
