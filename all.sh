#!/usr/bin/env bash 
# gnome-terminal -x bash -c "./pointcloud_to_laserscan_processing.sh;exec bash;"
# gnome-terminal -x bash -c "./skyplot_visualization.sh;exec bash;"
gnome-terminal -x bash -c "./openbag.sh;exec bash;"
gnome-terminal -x bash -c "./object_Detection.sh;exec bash;"
gnome-terminal -x bash -c "./tf.sh;exec bash;"
gnome-terminal -x bash -c "./puSkyplot.sh;exec bash;"
gnome-terminal -x bash -c "./puNlosExclusionProcess.sh;exec bash;"
gnome-terminal -x bash -c "./puGNSSEvaluation.sh;exec bash;"

