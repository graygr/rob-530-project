% File: main.m
% Author: Gregor Limstrom <limstrom@umich.edu>
% Purpose: Open data and explore format for 530 project

%% Import data from MRCLAM_1 - Defunct, they provide load and sample code
% Set pwd
% TODO: Change to match your filepath
cd("C:\Users\Lihuashu\Desktop\ROB530\rob-530-project")

% Load data
run('data\MRCLAM1\loadMRCLAMdataSet.m')

% Sample data
run('Tools\sampleMRCLAMdataSet.m')

%% Particle filter for localization

% Update based on measurements of landmarks and other robots

% Ask other robot location, their estimated accuracy rating (no. landmarks
% in their view)
% % Some chance of failure (packet loss)
% % 
 
%% Unscented Kalman Filter
run('src\runUKF.m')

% run('Tools\animateMRCLAMdataSet.m')


