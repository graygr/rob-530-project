% File: mainUKF.m
% Author: Huashu Li <lihs@umich.edu>
% Purpose: Open data and explore format for 530 project

close all;
clearvars -except numSteps useGTOnly useLandmarksOnly useTrustFactor trustFactorTime results* filterName

disp("Running UKF");

waitbar_h = waitbar(0,'Waitbar UKF');

srcFolderPath = pwd;
addpath(srcFolderPath)
addpath UKF_helpers
addpath lib
cd("../");
topLevelPath = pwd;

cd(topLevelPath)

% Load data
run('data\MRCLAM1\loadMRCLAMdataSet.m')

% Sample data
run('sampleMRCLAMdataSet.m')

%% Particle filter for localization

% Update based on measurements of landmarks and other robots

% Ask other robot location, their estimated accuracy rating (no. landmarks
% in their view)
% % Some chance of failure (packet loss)
% % 
 
%% Unscented Kalman Filter
cd(srcFolderPath)
run('runUKF.m')

% run('Tools\animateMRCLAMdataSet.m')


