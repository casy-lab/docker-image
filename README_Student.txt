For the students:
1) Pull the docker image repo with "git clone https://github.com/casy-lab/docker-image.git"
2) Pulling the remote Docker with "docker pull lollogent/uav-base:latest"
2BIS) In alternative you can build it from source with "./build-image.sh uav-base"
      WARNING: Building it from source may takes so many time (around 2/3 hours) !!!
3) Pull (or create) the ROS workspace with "git clone https://github.com/casy-lab/student-catkin-ws"
3a) Create a new branch with "git checkout -b devel/$STUDENT NAME$"
4) Pull the PX4 Autopilot with "git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
4b) Checkout the autopilot to a specific version, let's say v1.12.0 with "git checkout tags/v1.12.0"
4c) Update all submodules with "git submodule update --recursive"
5) Navigate inside the file run-image.sh and change rows 21 and 22 with you own directories, e.g.
   --volume=/Users/lorenzogentilini/Git/Others/CatkinWorkspace:/home/docker-dev/catkin_ws ==> --volume=/Users/mariorossi/Folder/Folder/:/home/docker-dev/catkin_ws
   WARNING: In general you should write --volume=$DIRECTORY TO CATKIN_WS$:/home/docker-dev/catkin_ws
   WARNING: In general you should write --volume=$DIRECTORY TO AUTOPILOT$:/home/docker-dev/PX4-Autopilot \
5) Run the docker container with "./run-image.sh uav-base"

Steps to enable depth registration in Docker simulation (Do it only once):
1) 

To launch the simulation:
1) Run the simulation with "roslaunch px4 mavros_posix_sitl.launch"

When you end your work/thesis, you should memorize all the developed code in the remote repository, using the following command:
"git push origin devel/$STUDENT NAME$"

N.B: You can open how many shells you want bu running on different terminals the same command "./run-image.sh uav-base".
     To close the docker run "exit", then you can close the terminal.
     If you require other dependencies, please contact me (lorenzo.gentilini6@unibo.it).
     All names must not contains spaces, you can use strange symbols as -/_&% and so on...