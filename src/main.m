clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% User define the following values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set the number of time steps to run. Each time step is 0.02 seconds
numSteps = 30000;

%select a filter to run. Options are: "EKF","PF","UKF" or "ALL"
filterName="ALL"; 

% Specify the amount of time for the trust factor
trustFactorTime=20; %seconds

% please select an operating mode by specifying an option
% Option : "useGT", "useLandmarksOnly", or "useTrustFactor"
mode = "useGT";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

useGTOnly=false;
useLandmarksOnly=false;
useTrustFactor=false;
if mode == "useGT"
    useGTOnly=true;
elseif mode == "useLandmarksOnly"
    useLandmarksOnly=true;
elseif mode == "useTrustFactor"
    useTrustFactor=true;
else 
    error("Unexpected Mode. Mode options are 'useGT', 'useLandmarksOnly', or 'useTrustFactor'");
end


if filterName == "EKF"
    run('mainEKF.m');
elseif filterName == "UKF"
    run('mainUKF.m');
elseif filterName == "PF"
    run('mainPF.m');
elseif filterName == "ALL"
    run('mainEKF.m');
    run('mainPF.m');
    run('mainUKF.m');
else
    error("Unexpected filter. Filter options are 'EKF,'PF',or 'UKF'");
end
clearvars -except numSteps useGTOnly useLandmarksOnly useTrustFactor trustFactorTime results* filterName
    

%% plotting

filtersToPlot = [filterName];
if filterName == "ALL"
    filtersToPlot = ["EKF" "PF" "UKF"];
end
for filterIndex=1:length(filtersToPlot)
    filterName = filtersToPlot(filterIndex);
    figure();
    hold on;
    for i=1:5
        cmd = strcat('results = results', num2str(i), filterName,';');
        eval(cmd);
        plot(1:numSteps,results(8,1:numSteps));   
    end

    if useLandmarksOnly == true
        title([filterName 'Landmarks Only']);
    elseif useGTOnly == true
        title([filterName 'Use Robot GT Measurement']);
    elseif useTrustFactor == true
        astring = strcat('Use Robot Est w/ ',num2str(trustFactorTime),' sec trust barrier');
        title([filterName astring]);
    end

    ylim([0 9]);
    xlabel("Timestep");
    ylabel("Distance Error");
    legend("Robot 1","Robot 2","Robot 3","Robot 4","Robot 5");
    
    distanceRMSE = sqrt(sum(results(8,1:numSteps).^2)/numSteps);
    stdDeviation = std(results(8,1:numSteps));
    astring = strcat(filterName,': Robot: ',num2str(i),' distance RMSE [', num2str(distanceRMSE), '] std deviation [', num2str(stdDeviation), '].');
    disp(astring);
end




