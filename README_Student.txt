For the students:
1) Pull the docker image repo with "git clone https://github.com/casy-lab/docker-image.git"
2) Pulling the remote Docker with "docker pull lollogent/uav-base:latest" (https://hub.docker.com/r/lollogent/uav-base/tags)
2BIS) In alternative you can build it from source with "./build-image.sh uav-base"
      WARNING: Building it from source may take so much time (around 2/3 hours) !!!
3) Pull (or create) the ROS workspace with "git clone https://github.com/casy-lab/student-catkin-ws"
3a) Create a new branch with "git checkout -b devel/$STUDENT NAME$"
4) Pull the PX4 Autopilot with "git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
4b) Checkout the autopilot to a specific version, let's say v1.12.0 with "git checkout tags/v1.12.0"
4c) Update all submodules with "git submodule update --recursive"
4d) Follow the "PX4 Addons" items before proceeding on
5) Navigate inside the file run-image.sh and change rows 21 and 22 with you own directories, e.g.
   --volume=/Users/lorenzogentilini/Git/Others/CatkinWorkspace:/home/docker-dev/catkin_ws ==> --volume=/Users/mariorossi/Folder/Folder/:/home/docker-dev/catkin_ws
   WARNING: In general you should write --volume=$DIRECTORY TO CATKIN_WS$:/home/docker-dev/catkin_ws
   WARNING: In general you should write --volume=$DIRECTORY TO AUTOPILOT$:/home/docker-dev/PX4-Autopilot \
6) Run the docker container with "./run-image.sh uav-base"
7) Follow the steps to launch the simulation

PX4 Addons procedure:
1) Pull the PX4 Addons repo with "git clone https://github.com/casy-lab/addons-px4"
2) Copy and paste the content of "addons-px4/models" to "PX4-Autopilot/Tools/sitl_gazebo/models"
3) Copy and paste the content of "addons-px4/world" to "PX4-Autopilot/Tools/sitl_gazebo/worlds"
4) Copy and paste the file "addons-px4/iris.sdf.jinja" to "PX4-Autopilot/Tools/sitl_gazebo/models/iris"
5) Navigate inside the file "PX4-Autopilot/launch/mavros_posix_sitl.launch"
5b) Change the row 19 from "<arg name="gui" default="true"/>" to "<arg name="gui" default="false"/>"
6) To change the simulation environment change the row 15 from "<arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>" to "<arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/$YOUR WORLD$.world"/>"

To launch the simulation:
1) Navigate inside PX4-Autopilot folder with "cd PX4-Autopilot"
2) Compile the autopilot with "DON_RUN=1 make px4_sitl_default gazebo"
3) Run the simulation with "roslaunch px4 mavros_posix_sitl.launch"
4) Launch your own ROS nodes

When you end your work/thesis, you should memorize all the developed code in the remote repository, using the following command:
"git push origin devel/$STUDENT NAME$"

N.B: You can open how many shells you want by running on different terminals the same command "./run-image.sh uav-base".
     To close the docker run "exit", then you can close the terminal.
     If you require other dependencies, please contact me (lorenzo.gentilini6@unibo.it).
     All names must not contain spaces, you can use strange symbols as -/_&% and so on...