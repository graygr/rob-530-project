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
%% Particle filter for localization

% Update based on measurements of landmarks and other robots

% Ask other robot location, their estimated accuracy rating (no. landmarks
% in their view)
% % Some chance of failure (packet loss)
% % 
 