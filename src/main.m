% File: main.m
% Author: Gregor Limstrom <limstrom@umich.edu>
% Purpose: Open data and explore format for 530 project

%% Import data from MRCLAM_9
% Set pwd
% TODO: Change to match your filepath
cd("C:\Users\Gregor Limstrom\Documents\masters\NAVARCH 568\project_red")

% Read in landmark data
% Data format: landmark id - x - y - ?? - ??
landmark_gt = readtable("MRCLAM9/Landmark_Groundtruth.dat");

% Read in barcode table
% Data format: landmark id - barcode id
barcodes = readtable("MRCLAM9/Barcodes.dat");

% Read in GT data
% Data Format: timestamp - x - y - theta

robot_1_gt = readtable("MRCLAM9/Robot1_Groundtruth.dat");
robot_2_gt = readtable("MRCLAM9/Robot2_Groundtruth.dat");
robot_3_gt = readtable("MRCLAM9/Robot3_Groundtruth.dat");
robot_4_gt = readtable("MRCLAM9/Robot4_Groundtruth.dat");
robot_5_gt = readtable("MRCLAM9/Robot5_Groundtruth.dat");

% Read in Odometry data
% Data Format: timestamp - dx - dy

robot_1_odo = readtable("MRCLAM9/Robot1_Odometry.dat");
robot_2_odo = readtable("MRCLAM9/Robot2_Odometry.dat");
robot_3_odo = readtable("MRCLAM9/Robot3_Odometry.dat");
robot_4_odo = readtable("MRCLAM9/Robot4_Odometry.dat");
robot_5_odo = readtable("MRCLAM9/Robot5_Odometry.dat");

% Read in Measurement data
% Data Format: timestamp - landmark id - range - bearing

robot_1_meas = readtable("MRCLAM9/Robot1_Measurement.dat");
robot_2_meas = readtable("MRCLAM9/Robot2_Measurement.dat");
robot_3_meas = readtable("MRCLAM9/Robot3_Measurement.dat");
robot_4_meas = readtable("MRCLAM9/Robot4_Measurement.dat");
robot_5_meas = readtable("MRCLAM9/Robot5_Measurement.dat");

%% Particle filter for localization

% Update based on measurements of landmarks and other robots

% Ask other robot location, their estimated accuracy rating (no. landmarks
% in their view)
% % Some chance of failure (packet loss)
% % 
 