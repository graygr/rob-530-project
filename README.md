# Multi-Robot Localization with Relative Observations Gated by Trust Factor
Cameron Kabacinski, Huashu Li, Gregor Limstrom

For NAVARCH 568 / ROB 530 Final Project

In this project, we created several different filters to perform cooperative localization using fusion of odometry and landmark measurements. We wrote and tested an Extended Kalman Filter, a Particle Filter, and an Unscented Kalman Filter in order to find the program most robust to real world measurement noise and measurement starvation. 

In order to model a trust system for the robots, we measured the number of update ticks since the last fully observable state for each robot, and only updated the corresponding state when we determined that a robot was well versed enough in its own location to localize off of and correct the odometry drift. This resulted in large improvements for the Particle Filter, but the UKF still performed the best through the trial. 

Future work to explore

Link to Presentation: [Link](https://www.youtube.com/watch?v=r0BZHNcIqdw)

Link to Presentation Slides: [Link](https://docs.google.com/presentation/d/1VIBYjhWs4f2Vct2t7GQAV90KE2pGuMCUh0T6ZfCsncY/edit?usp=sharing)

# Environment Setup
We used Matlab R2021a
  
# Dataset
We used the [UTIAS Multi-Robot Cooperative Localization and Mapping] Dataset(http://asrl.utias.utoronto.ca/datasets/mrclam/index.html), specifically the first dataset for developement and testing purposes.
  
# Usage
To switch between the filter, switch to the corresponding branch. EKF is on master, PF and UKF are on their corresponding branches.

From there, run the main.m file in each branch in order to execute the filter. 

TODO: Combine all the branches and make it nice and easy to call different filters.
