# yaprofi-team1-2020
Team 1 repo for Tello &lt;-> ROS stuff


Works with on Tello-ROS bridge https://github.com/kirillin/tello_driver

## Some notes on drone control
1. Explanatory drawing of the reference frames 
<img src="drone_frames.png" width="600">
2. _Twist_ of `/tello/cmd_vel` topic is expressed  in _{body}_ frame
3. _Odometry_ of `/tello/odom` topic  is expressed in _{odom}_ frame