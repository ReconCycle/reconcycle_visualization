The docker-compose reconcycle-base, and within it the robot_rviz, run this package so as to visualize the reconcycle table layout.

The command run within the docker compose is:
command: stdbuf -o L roslaunch --wait reconcycle_description display_real_qundis.launch

Which means that you should check out the display_real_qundis launch file to see what's happening.
 
