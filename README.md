## To run code

1. You need the right Docker enviorment, see Lecture L1
2. The docker daemon needs to bee running, this can be done by opening the docker desktop application
3. In the terminal navigate to the docker file and do ```./run_vnc.sh ``` and ```./run_docker_vnc.sh ```
4. Start a TMUX session by doing ```tmux new -s session```
5. Now do ```roscore```, this needs to be running, therfore do ```ctrl + b``` and ```shift + 2```to open a new termnial
6. Do ```catkin_make``` at cd ~/catkin_ws
7. Navigate to first_project and to ```roslaunch first_project launch.launch```
8. Open another terminal and navigate to the src folder. Do ```rosbag play --clock robotics.bag```. The bag is now running and you will se ENU coordinates in the privous terminal
9. Open a new termnail, do ```rviz```, go to localhost in browser, go to vnc.html and connect, here you will see RViz
10. In RViz do add-By topic-Odometry-Ok, then change "Fixed frame" to "/odom". Then change the Odometry shape to Axes and zoom out 
