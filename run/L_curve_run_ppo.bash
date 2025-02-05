source /opt/ros/kinetic/setup.bash
source devel/setup.bash
export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:./devel/lib

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/catkin_ws/devel/lib' >> ~/.bashrc
source ~/.bashrc

roscore &

catkin_make && argos3 -c src/tensorswarm/argos_worlds/l_curve.argos &
argos3 -c src/tensorswarm/argos_worlds/l_curve.argos &
argos3 -c src/tensorswarm/argos_worlds/l_curve.argos &
argos3 -c src/tensorswarm/argos_worlds/l_curve.argos &

python3 src/tensorswarm/scripts/new/RunExperimentLCurve.py