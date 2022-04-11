% File: main.m
% Author: Gregor Limstrom <limstrom@umich.edu>, 
% Cameron Kabacinski <camkab@umich.ed>
% Purpose: Open data and explore format for 530 project

clc;
close all;
clearvars -except mainFilePath 
% Set pwd
if ~exist('mainFilePath')
    mainFilePath = pwd;
end
% change directory to top level so we can access data
cd("../")
%% Import data from MRCLAM_9 - Defunct, they provide load and sample code
% Load data
% run('data\MRCLAM9\loadMRCLAMdataSet.m')
run('data\MRCLAM1\loadMRCLAMdataSet.m')

% Sample data
run('Tools\sampleMRCLAMdataSet.m')

%% Animate data
%run('Tools\animateMRCLAMdataSet.m')

 
cd(mainFilePath)
%% EKF filter for localization
close all;
robotsToRun = [1 2 3 4 5];
% robotsToRun = [5 ];
% Makes an assumption that updates should keep the robot close to where its
% previus pose was, but this is flawed because if we drift away then we
% have no hopes to recover, so there should be a smarter way to decide if
% we reject or accept an update...
rejectDistanceThreshold = 0.3;
numSteps = 8000;%length(Robot1_Groundtruth)/8; %number of steps from dataset to run
useGTForObservedRobots = true;
useObservationsToCorrect = true;
plotStatistics = false;
beta(1) = deg2rad(25);
beta(2) = 1.2;%0.3;

run('runEKF.m');
useObservationsToCorrect = false;
run('runEKF.m');

% Update based on measurements of landmarks and other robots

% Ask other robot location, their estimated accuracy rating (no. landmarks
% in their view)
% % Some chance of failure (packet loss)
% % 
 