
% User define the following values
numSteps = 2000;
filter="PF"; 
useGTOnly=false;
useLandmarksOnly=false;
useTrustFactor=false;
trustFactorTime=20; %seconds

%%%%%



    if filter == "EKF"
        run('mainEKF.m');
    elseif filter == "UKF"
    elseif filter == "PF"
        run('mainPF.m');
    else
        error("Unexpected filter. Usage mainHelper(numStemps, filter), where filter is 'EKF,'PF',or 'UKF'");
    end
