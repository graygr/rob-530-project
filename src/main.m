% File: main.m
% Author: Gregor Limstrom <limstrom@umich.edu>, 
% Cameron Kabacinski <camkab@umich.ed>
% Purpose: Open data and explore format for 530 project

clc;
close all;
clearvars;
srcFolderPath = pwd;
cd("../");
topLevelPath = pwd;
cd("collectedData");
collectedDataPath = pwd;

runDataFolders = ["MRCLAM1" "MRCLAM9"];

for currDataIndex=1:length(runDataFolders)
    cd(topLevelPath)
    clearvars -except currDataIndex srcFolderPath topLevelPath collectedDataPath runDataFolders
    close all
    %Load and sample data
    cmd = strcat('run(''data\', runDataFolders(currDataIndex), '\loadMRCLAMdataSet.m'');');
    eval(cmd);
    run('Tools\sampleMRCLAMdataSet.m')
    % run('Tools\animateMRCLAMdataSet.m') % Animate data
    cd(srcFolderPath)
    
    % EKF filter for localization
    robotsToRun = [1 2 3 4 5];
    numSteps = 30000;%length(Robot1_Groundtruth); %number of steps from dataset to run
    plotStatistics = false;
    plotObservationLines = [false false false]; %show when observations were used in the [x y theta] plots
    % The measurements may have the wrong landmark! Ignore
    % the observation if the perceived landmark range and
    % bearing is more than landmarkDistanceThreshold 
    landmarkDistanceThreshold = 1.0; 
    alphas = [  0.25 0.05 ...
                0.25 0.5 ...
                0.25 0.05].^2;
    beta = [deg2rad(25) 25];
    
    % Collect data on localizing with landmarks and robot ground truth
    useObservationsToCorrect = true;
    useLandmarksOnly = false;
    useGTForObservedRobots = true;
    useEstimateForObservedRobots = ~useGTForObservedRobots;
    run('runEKF.m');
    close all;
    cd(collectedDataPath);
    cmd = strcat('save("', runDataFolders(currDataIndex), '_EKF_landmarksAndRobotGT.mat");');
    eval(cmd);
    cd(srcFolderPath);
    
    %Collect data on localizing with landmark data only
    useLandmarksOnly = true;
    run('runEKF.m');
    close all;
    cd(collectedDataPath);
    cmd = strcat('save("', runDataFolders(currDataIndex), '_EKF_landmarksOnly.mat");');
    eval(cmd);
    cd(srcFolderPath);
    
    % Collect data on localizing by predicition only
    useObservationsToCorrect = false;
    run('runEKF.m');
    close all;
    cd(collectedDataPath);
    cmd = strcat('save("', runDataFolders(currDataIndex), '_EKF_predictionOnly.mat");');
    eval(cmd);   
    cd(srcFolderPath);
    
end






%%

%%
      
% run('runEKF.m');
% save("CLAM1_correctionsUsingGt.mat");
% useObservationsToCorrect = false;
% run('runEKF.m');
% save("CLAM1_predictionOnly.mat");
% 
% 
% % now run the other dataset
% close all;
% clearvars -except mainFilePath topLevel alphas beta landmarkDistanceThreshold plotObservationLines ...
%     robotsToRun useGTForObservedRobots plotStatistics
% 
% cd(topLevel);
% run('data\MRCLAM9\loadMRCLAMdataSet.m')
% run('Tools\sampleMRCLAMdataSet.m')
% cd(mainFilePath)
% 
% numSteps = length(Robot1_Groundtruth); %number of steps from dataset to run
% 
% useObservationsToCorrect = true;
% 
% run('runEKF.m');
% save("CLAM1_correctionsUsingGt.mat");
% useObservationsToCorrect = false;
% run('runEKF.m');
% save("CLAM1_predictionOnly.mat");
% 

 

 