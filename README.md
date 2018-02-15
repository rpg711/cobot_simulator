# cobot_simulator

Add  to path
`export ROS_PACKAGE_PATH=PATH_TO_DIRECTORY:$ROS_PACKAGE_PATH`

Move into the cobot_linux directory
`cd PATH_TO_DIRECTORY/cobot_linux`

Build the messages directory
`make messages -j 6`

Build the program

`make -j 6`

Run

`./bin/cobot_simulator`

Publishes a laser scan message, and listens to a 
drive message.

Uses the first map in the file maps/atlas.txt.
