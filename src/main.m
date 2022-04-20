clear all;
close all;
clc;
% User define the following values
numSteps = 800;
filter="ALL"; 
useGTOnly=true;
useLandmarksOnly=false;
useTrustFactor=false;
trustFactorTime=20; %seconds

%%%%%


if filter == "EKF"
    run('mainEKF.m');
elseif filter == "UKF"
    run('mainUKF.m');
elseif filter == "PF"
    run('mainPF.m');
elseif filter == "ALL"
    run('mainEKF.m');
    run('mainPF.m');
    run('mainUKF.m');
else
    error("Unexpected filter. Usage mainHelper(numStemps, filter), where filter is 'EKF,'PF',or 'UKF'");
end
clearvars -except numSteps useGTOnly useLandmarksOnly useTrustFactor trustFactorTime results*


% if filter == "EKF"
%     run('mainEKF.m');
% elseif filter == "UKF"
% %     run('mainUKF.m');
% elseif filter == "PF"
%     run('mainPF.m');
% elseif filter == "ALL"
%     run('mainEKF.m');
%     run('mainPF.m');
% %     run('mainUKF.m');
% else
%     error("Unexpected filter. Usage mainHelper(numStemps, filter), where filter is 'EKF,'PF',or 'UKF'");
% end

% eval(['results = results' num2str(id) 'EKF;'])
% distanceRMSE = sqrt(sum(results(8,1:numSteps).^2)/numSteps);
% stdDeviation = std(results(8,1:numSteps));
% disp(['distance RMSE [' num2str(distanceRMSE) '] std deviation [' num2str(stdDeviation) '].']);






% clc;
% close all;
% clearvars -except numSteps useGTOnly useLandmarksOnly useTrustFactor trustFactorTime results*
% 
% 
% 
% srcFolderPath = pwd;
% addpath(srcFolderPath)