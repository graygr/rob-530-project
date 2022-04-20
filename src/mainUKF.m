% File: mainUKF.m
% Author: Huashu Li <lihs@umich.edu>
% Purpose: Open data and explore format for 530 project

%% Import data from MRCLAM_1 - Defunct, they provide load and sample code

clc;clear; close all;


srcFolderPath = pwd;
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

