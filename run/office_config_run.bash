source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:./devel/lib

roscore &

catkin_make && argos3 -c src/tensorswarm/argos_worlds/office_config.argos &
argos3 -c src/tensorswarm/argos_worlds/office_config.argos &
argos3 -c src/tensorswarm/argos_worlds/office_config.argos &
argos3 -c src/tensorswarm/argos_worlds/office_config.argos &

python3 src/tensorswarm/scripts/new/RunExperimentLCurve.py