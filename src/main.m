clear all;
close all;
clc;
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% User define the following values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set the number of time steps to run. Each time step is 0.02 seconds
numSteps = 30000;

%select a filter to run. Options are: "EKF","PF","UKF" or "ALL"
filterName="ALL"; 

% Specify the amount of time for the trust factor
trustFactorTime=10; %seconds

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
numCols = 1;
if filterName == "ALL"
    filtersToPlot = ["EKF" "PF" "UKF"];
    numCols = 3;
end
figure()
currSubplot=1;

if useLandmarksOnly == true
    sgtitle('Landmarks Only');
elseif useGTOnly == true
    sgtitle('Use Robot GT Measurement');
elseif useTrustFactor == true
    astring = strcat('Use Robot Est w/ ',num2str(trustFactorTime),' sec trust barrier');
    sgtitle(astring);
end

for filterIndex=1:length(filtersToPlot)
    filterNameCurr = filtersToPlot(filterIndex);
    subplot(1,numCols,currSubplot);
    currSubplot = currSubplot + 1;
    hold on;
    for i=1:5
        cmd = strcat('results = results', num2str(i), filterNameCurr,';');
        eval(cmd);
        plot(1:numSteps,results(8,1:numSteps));   
        distanceRMSE = sqrt(sum(results(8,1:numSteps).^2)/numSteps);
        stdDeviation = std(results(8,1:numSteps));
        astring = strcat(filterNameCurr,': Robot: ',num2str(i),' distance RMSE [', num2str(distanceRMSE), '] std deviation [', num2str(stdDeviation), '].');
        disp(astring);
    end

    if useLandmarksOnly == true
        title(filterNameCurr);% 'Landmarks Only']);
    elseif useGTOnly == true
        title(filterNameCurr);% 'Use Robot GT Measurement']);
    elseif useTrustFactor == true
        astring = strcat('Use Robot Est w/ ',num2str(trustFactorTime),' sec trust barrier');
        title(filterNameCurr);% astring]);
    end

    ylim([0 9]);
    
    xlabel("Timestep");
    if filterIndex == 1
        ylabel("Distance Error m");
    end
    if filterIndex == 3 || (filterIndex ==1 && numCols == 1)
        legend("Robot 1","Robot 2","Robot 3","Robot 4","Robot 5");
    end
    
end




