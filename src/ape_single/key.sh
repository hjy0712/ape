#! /bin/bash
source /home/ape/.bashrc
source /opt/ros/noetic/setup.bash
rosclean purge -y
amixer set -c 0 Master 1000
gnome-terminal -t "serial" -x bash -c "cd /home/ape && echo '123123' | sudo -S chmod 777 /dev/ttyUSB0; exec bash;"
sleep 2
wait
gnome-terminal -t "ros_server" -x bash -c "source /home/ape/.bashrc && source /opt/ros/noetic/setup.bash && source /home/ape/APE_Application/devel/setup.bash && roslaunch ape_single start_server.launch; exec bash;"
sleep 5
wait
gnome-terminal -t "python_server" -x bash -c "source /home/ape/.bashrc && source /opt/ros/noetic/setup.bash && source /home/ape/APE_Application/devel/setup.bash && cd /home/ape/APE_Application/src/ape_single/script && /home/ape/miniconda3/envs/flask/bin/python main.py; exec bash;"
wait
exit 0
