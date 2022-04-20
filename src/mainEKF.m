% File: mainEKF.m
% Author:  Cameron Kabacinski <camkab@umich.ed>
% Purpose: Open data and explore format for 530 project

% clc;
% close all;
% clearvars;

srcFolderPath = pwd;
addpath lib
addpath EKF_helpers
cd("../");
topLevelPath = pwd;
cd("collectedData");
collectedDataPath = pwd;

% runDataFolders = ["MRCLAM1" "MRCLAM9"];
runDataFolders = ["MRCLAM1"];
saveOutput = false;

for currDataIndex=1:length(runDataFolders)
    cd(topLevelPath)
    clearvars -except currDataIndex srcFolderPath topLevelPath collectedDataPath runDataFolders numSteps
    close all
    %Load and sample data
    cmd = strcat('run(''data\', runDataFolders(currDataIndex), '\loadMRCLAMdataSet.m'');');
    eval(cmd);
    cd(srcFolderPath)
    run('sampleMRCLAMdataSet.m')
    % run('animateMRCLAMdataSet.m') % Animate data
    
    % EKF filter for localization
    robotsToRun = [1 2 3 4 5];
    numSteps = 1000;%length(Robot1_Groundtruth); %number of steps from dataset to run
    plotStatistics = false;
    plotObservationLines = [false false false]; %show when observations were used in the [x y theta] plots
    % The measurements may have the wrong landmark! Do something about the 
    % the observation if the perceived landmark range and
    % bearing is more than landmarkDistanceThreshold 
    landmarkDistanceThreshold = 1.0; 
    % use likely landmarkDistance threshold to see if there are any other
    % landmarks that would agree with the observation by the proposed likely landmark
    % being within the likely distance threshold of the observation
    likelyLandmarkDistanceThreshold = 0.5; 
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
    break
    close all;
    if saveOutput == true
        cd(collectedDataPath);
        cmd = strcat('save("', runDataFolders(currDataIndex), '_EKF_landmarksAndRobotGT.mat");');
        eval(cmd);
        cd(srcFolderPath);
    end
    
    %Collect data on localizing with landmark data only
    useLandmarksOnly = true;
    run('runEKF.m');
    close all;
    if saveOutput == true
        cd(collectedDataPath);
        cmd = strcat('save("', runDataFolders(currDataIndex), '_EKF_landmarksOnly.mat");');
        eval(cmd);
        cd(srcFolderPath);
    end
    
    %Collect data on localizing with landmarks and estimates of robots with
    %trustfactor
    useObservationsToCorrect = true;
    useLandmarksOnly = false;
    useGTForObservedRobots = false;
    useEstimateForObservedRobots = ~useGTForObservedRobots;
    run('runEKF.m');
    close all;
    if saveOutput == true
        cd(collectedDataPath);
        cmd = strcat('save("', runDataFolders(currDataIndex), '_EKF_landmarksAndRobotEstwTrust.mat");');
        eval(cmd);
        cd(srcFolderPath);
    end
    
    % Collect data on localizing by predicition only
    useObservationsToCorrect = false;
    run('runEKF.m');
    close all;
    if saveOutput == true
        cd(collectedDataPath);
        cmd = strcat('save("', runDataFolders(currDataIndex), '_EKF_predictionOnly.mat");');
        eval(cmd);   
        cd(srcFolderPath);
    end
    
end

 