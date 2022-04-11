

% robotsToRun = [1 2 3 4 5];
% robotsToRun = [1 ];
% rejectDistanceThreshold = 0.35;
% numSteps = 8000;%length(Robot1_Groundtruth)/8; %number of steps from dataset to run
% useGTForObservedRobots = true;
% useObservationsToCorrect = true;
%options if you want to show the time steps when observations were used in the [x y theta] plots
plotObservationLines = [false false false]; 
plotSigmaEllipses = false;
% plotStatistics = true;

disp("Running EKF with robots:");
disp(robotsToRun);
tic;
waitbar_h = waitbar(0,'Waitbar');

%Assume for now that all robots will use same alphas and beta
% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [  0.25 0.05 ...
            0.25 0.5 ...
            0.25 0.05].^2; % variance of noise proportional to alphas
% alphas = [  0.00025 0.00005 ...
%             0.0025 0.05 ...
%             0.0025 0.0005].^2; % variance of noise proportional to alphas
% Standard deviation of Gaussian sensor noise (independent of distance)
% beta(1) = deg2rad(10);
% beta(2) = 100;

%note that delta T will become part of the g function and its jacobians
%in our HW5 we treated delta t as 1. so we should try using it as not 1
deltaT = 0.02; %it probably will be 0.02 based on 50Hz odometry commands

systems = {};
filters = {};
for i=1:length(robotsToRun)
    id = robotsToRun(i);
    %Get the initialStateMean and Cov for each robot, variables will use the
    %name and tack on their id
    eval(['initialStateMean' num2str(id) ' = Robot' num2str(id) '_Groundtruth(1,2:end)'';'])
    eval(['initialStateCov' num2str(id) ' = eye(3);'])
    sys = system_initialization(alphas, beta, deltaT);
    systems{i} = sys;
    eval(['filters{i}=filter_initialization(sys,  initialStateMean' num2str(id) ',  initialStateCov' num2str(id) ', "EKF", deltaT);'])
    
    eval(['observationsUsedatIndex' num2str(id) ' = [];'])
    eval(['robotPose' num2str(id) ' = zeros(numSteps,3);'])
    eval(['robotSigmas' num2str(id) ' = {};'])
    eval(['lastvalidMeasurementIndex' num2str(id) ' = 1;'])
    eval(['rejectedUpdates' num2str(id) ' = 0;'])
    if plotStatistics
        eval(['results' num2str(id) ' = zeros(8,numSteps);'])
    end 
end

% subect number ids are barcode values, and need to be mapped to which
% robot or landmark they are
keySet = Barcodes(:,2);
valueSet = Barcodes(:,1);
subjectNumToIDMAP = containers.Map(keySet,valueSet);

%create map containers to get the landmark positions
keySet2 = Landmark_Groundtruth(:,1);
xValueSet = Landmark_Groundtruth(:,2);
yValueSet = Landmark_Groundtruth(:,3);
landmarkIDToXMAP = containers.Map(keySet2,xValueSet);
landmarkIDToYMAP = containers.Map(keySet2,yValueSet);

%%
lastPerc = 0;
for t = 1:numSteps
    
    perc = t/numSteps;
    if (abs(perc-lastPerc)>.05)
        lastPerc = perc;
        waitbar(perc,waitbar_h,sprintf('%f%% along...',perc*100))
    end

    eval(['currTimeCheck = Robot' num2str(robotsToRun(1)) '_Odometry(t,1);'])
    for i=1:length(robotsToRun)
        id = robotsToRun(i);

        noiseFreeMotionCommand = zeros(1,3);
        eval(['noiseFreeMotionCommand(1:2) = Robot' num2str(id) '_Odometry(t,2:3);'])

        % [Trans_vel,Angular_vel,gamma]' noisy control command
        noisyMotionCommand = sampleOdometry(noiseFreeMotionCommand,alphas);
        %add a small amount of noise so that v,w are never exactly zero
        noisyMotionCommand = noisyMotionCommand + 0.0001.*randn(size(noisyMotionCommand));
    
        % Time should be synchronized for all robots considered
        % the observations from the dataset are already considered noisy
        eval(['currTime = Robot' num2str(id) '_Odometry(t,1);'])
        if currTime ~= currTimeCheck
            error("Time mismatch in odometry data");
        end
    

        measurements = [];
        measurementsIndex = 1;
        eval(['lastvalidMeasurementIndex = lastvalidMeasurementIndex' num2str(id) ';'])
        eval(['endMeasurementIndex = size(Robot' num2str(id) '_Measurement,1);'])
        for j=lastvalidMeasurementIndex:endMeasurementIndex
            eval(['measurementTime = Robot' num2str(id) '_Measurement(j,1);'])
    %         msg = ['J:',num2str(j),' CurrTime: (',num2str(t),',',num2str(currTime),') MeasureTime: ',num2str(measurementTime)];
    %         disp(msg);
            if abs(measurementTime - currTime) <= 0.001

                eval(['measurements(measurementsIndex,:) = Robot' num2str(id) '_Measurement(j,2:end);'])
                measurementsIndex = measurementsIndex + 1;
            elseif measurementTime > currTime
                eval(['lastvalidMeasurementIndex' num2str(id) ' = max(j-3,1);'])
                break;
            end
        end
        %let us see if we actually got any measurements
        %measurements are   [subject barcode id1, range1, bearing1;
        %                    subject barcode id2, range2, bearing2; ...

        %observations will be [bearing1, bearing2, ...
        %                      range1,   range2, ...
        %                      xglobalpos1, xglobalpos2,...
        %                      yglobalpos1, yglobalpos2,... 
        %                      observed landmark/robot id1, "" id2
        %                      subject barcode id1, "" id2]
        observations = [];
        obsCol = 1;
        if ~isempty(measurements)     
            for j=1:size(measurements,1)
                idObserved = subjectNumToIDMAP(measurements(j,1));
                if idObserved < 6
                    %for now we are testing EKF with being able to get the
                    %exact position (ground truth) of observed robots and landmarks
                    if useGTForObservedRobots
        %                 disp(['observed robot: ',num2str(id)])
                        eval(['gt = Robot' num2str(idObserved) '_Groundtruth(' num2str(t) ',1:4);'])
                        if (gt(1) ~= currTime)
                            error("GT time of observed robot does not match curr time.");
                        end
                        observations(1, obsCol) = measurements(j,3); %get bearing from measurment
                        observations(2, obsCol) = measurements(j,2); %get range from measurment
                        observations(3, obsCol) = gt(2); %get x global pos of observed id
                        observations(4, obsCol) = gt(3); %get y global pos of observed id
                        observations(5, obsCol) = idObserved; %observed id
                        observations(6, obsCol) = measurements(j,1); %barcode id
                        obsCol = obsCol +1;
                    else
                        error("Not setup for other than GT positions");
                    end

                else
    %                 disp(['observed landmark: ',num2str(id)])
                    observations(1, obsCol) = measurements(j,3); %get bearing from measurment
                    observations(2, obsCol) = measurements(j,2); %get range from measurment
                    observations(3, obsCol) = landmarkIDToXMAP(idObserved); %get x global pos of observed id
                    observations(4, obsCol) = landmarkIDToYMAP(idObserved); %get y global pos of observed id
                    observations(5, obsCol) = idObserved; %observed id
                    observations(6, obsCol) = measurements(j,1); %barcode id
                    obsCol = obsCol +1;
                end
            end

        end
        observationsAvailable =  size(observations,2)>=2;%~isempty(observations) &&

        %if there is no motion command, then the robot should not be moving so
        %don't update filter
        % this may violate some dyanmics due to inertia, but should be fine
    %     zeroThreshold = 0.001;
    %     if ((abs(noiseFreeMotionCommand(1)) <= zeroThreshold) && ...
    %         (abs(noiseFreeMotionCommand(2)) <= zeroThreshold))
    %         %no motion
    %         robot1Pose(t,:)= filter.mu(1:3)';
    %         %countContinue = countContinue + 1;
    %         continue;
    %     end
    
       %we assume that prediction step will usually be decent, so we only
       %want to guarda gainst a bad correction step
       filters{i}.prediction(noisyMotionCommand);
       muPrev = filters{i}.mu_pred;
       sigmaPrev = filters{i}.Sigma_pred;
       
       if observationsAvailable && useObservationsToCorrect
           eval(['observationsUsedatIndex' num2str(id) ' = [observationsUsedatIndex' num2str(id) ' t];'])
           filters{i}.correction(observations);
       else
           filters{i}.setPredictionAsCurrent();
       end
       muAfter = filters{i}.mu;
       
       distance = sqrt((muPrev(1)-muAfter(1))^2+(muPrev(2)-muAfter(2))^2);
       if distance >= rejectDistanceThreshold
%            disp("Too large of jump, using mu/sigma before update");
           eval(['rejectedUpdates' num2str(id) ' = rejectedUpdates' num2str(id) '+1;'])
           filters{i}.mu = muPrev;
           filters{i}.Sigma = sigmaPrev;
       end
       
       eval(['robotPose' num2str(id) '(t,:) = filters{i}.mu(1:3)'';'])
       eval(['robotSigmas' num2str(id) '{t} = filters{i}.Sigma;'])
       eval(['GTMu = Robot' num2str(id) '_Groundtruth(t,2:4)'';'])
       
       eval(['results' num2str(id) '(:,t) = mahalanobis(filters{i}.mu,filters{i}.Sigma,GTMu);'])

    end
    
end
toc;
close(waitbar_h);
%% Plot
disp("Plotting results");
tic;

% close all;
numSteps = t;
for i=1:length(robotsToRun)
    id = robotsToRun(i);
    
    

    figure('units','normalized','outerposition',[0 0 1 1])
    if useObservationsToCorrect
        msg = ['Robot: ' num2str(id) ' EKF with prediction and correction'];
        sgtitle(msg);
        disp(msg);
    else
        msg = ['Robot: ' num2str(id) ' EKF with prediction only'];
        sgtitle(msg);
        disp(msg);
    end
    eval(['numRejected = rejectedUpdates' num2str(id) ';'])
    disp(['Robot: ' num2str(id) ' had ' num2str(numRejected) ' rejected updates.']);
    
    subplot(2,2,1);
    title("Robot x,y over time");
    hold on;
    
    robotEstimatedXdata = [];
    robotEstimatedYdata = [];
    robotEstimatedTdata = [];
    robotGTXdata = [];
    robotGTYdata = [];
    robotGTTdata = [];
    observationsUsedatIndex = [];
    eval(['robotEstimatedXdata = robotPose' num2str(id) '(1:numSteps,1);'])
    eval(['robotEstimatedYdata = robotPose' num2str(id) '(1:numSteps,2);'])
    eval(['robotEstimatedTdata = robotPose' num2str(id) '(1:numSteps,3);'])
    eval(['robotGTXdata = Robot' num2str(id) '_Groundtruth(1:numSteps,2);'])
    eval(['robotGTYdata = Robot' num2str(id) '_Groundtruth(1:numSteps,3);'])
    eval(['robotGTTdata = Robot' num2str(id) '_Groundtruth(1:numSteps,4);'])
    eval(['observationsUsedatIndex = observationsUsedatIndex' num2str(id) ';'])
    
    plot(robotEstimatedXdata,robotEstimatedYdata,'r.-');
    plot(robotGTXdata,robotGTYdata,'b.-');
    if plotSigmaEllipses
        for j=1:numSteps
            eval(['sigma=robotSigmas' num2str(id) '{j};'])
            draw_ellipse([robotEstimatedXdata(j);robotEstimatedYdata(j)], sigma(1:2,1:2),1);
        end
    end
    legend('Estimated pose','GroundTruth');

    subplot(2,2,2);
    title("theta data vs time");
    hold on;
    plot(1:numSteps,robotEstimatedTdata,'r.-');
    plot(1:numSteps,robotGTTdata,'b.-');
    if plotObservationLines(3) 
        for j=1:length(observationsUsedatIndex)
            xline(observationsUsedatIndex(j));
        end
    end
    
    subplot(2,2,3);
    title("Xdata vs time");
    hold on;
    plot(1:numSteps,robotEstimatedXdata,'r.-');
    plot(1:numSteps,robotGTXdata,'b.-');
    if plotObservationLines(1) 
        for j=1:length(observationsUsedatIndex)
            xline(observationsUsedatIndex(j));
        end
    end
    
    subplot(2,2,4);
    title("Ydata vs time");
    hold on;
    plot(1:numSteps,robotEstimatedYdata,'r.-');
    plot(1:numSteps,robotGTYdata,'b.-');
    if plotObservationLines(2) 
        for j=1:length(observationsUsedatIndex)
            xline(observationsUsedatIndex(j));
        end
    end

    eval(['results = results' num2str(id) ';'])
    distanceRMSE = sqrt(sum(results(8,1:numSteps).^2)/numSteps);
    stdDeviation = std(results(8,1:numSteps));
    disp(['Robot ' num2str(id) ': distance RMSE[' num2str(distanceRMSE) '] std deviation[' num2str(stdDeviation) '].']);
        
    if plotStatistics
        figure('units','normalized','outerposition',[0 0 1 1]);
        if useObservationsToCorrect
            sgtitle(['Statistics Robot ' num2str(id) ' EKF w/ prediction and correction [Distance RMSE ' num2str(distanceRMSE) ' ]']);
        else
            sgtitle(['Statistics Robot ' num2str(id) ' EKF w/ prediction only [Distance RMSE ' num2str(distanceRMSE) ' ]']);
        end
        rows = 2;
        cols = 3;
        currSubplot = 1;
        subplot(rows,cols,currSubplot);
        currSubplot = currSubplot + 1;
        title("Chi-square Statistics");
        hold on; 
        plot(results(1,1:numSteps), 'linewidth', 2)
        plot(7.81*ones(1,numSteps),'r', 'linewidth', 2)
        legend('Chi-square Statistics','p = 0.05 in 3 DOF', 'fontsize', 14, 'location', 'best')

        subplot(rows,cols,currSubplot);
        currSubplot = currSubplot + 1;
        title("3*sigma value: X");
        hold on;
        plot(results(2,1:numSteps), 'linewidth', 2);
        plot(results(5,1:numSteps),'r', 'linewidth', 2);
        plot(-1*results(5,1:numSteps),'r', 'linewidth', 2);
        ylabel('X', 'fontsize', 14);
        xlabel('Iterations', 'fontsize', 14);
        legend('Deviation from Ground Truth','3rd Sigma Contour', 'fontsize', 14, 'location', 'best');
        
        subplot(rows,cols,currSubplot);
        currSubplot = currSubplot + 1;
        title("3*sigma value Y");
        hold on; 
        plot(results(3,1:numSteps), 'linewidth', 2);
        plot(results(6,1:numSteps),'r', 'linewidth', 2);
        plot(-1*results(6,1:numSteps),'r', 'linewidth', 2);
        ylabel('Y', 'fontsize', 14);
        xlabel('Iterations', 'fontsize', 14);
        
        subplot(rows,cols,currSubplot);
        currSubplot = currSubplot + 1;
        title("3*sigma value: \theta");
        hold on; 
        plot(results(4,1:numSteps), 'linewidth', 2);
        plot(results(7,1:numSteps),'r', 'linewidth', 2);
        plot(-1*results(7,1:numSteps),'r', 'linewidth', 2);
        ylabel('\theta', 'fontsize', 14);
        xlabel('Iterations', 'fontsize', 14);
        
        subplot(rows,cols,currSubplot);
        currSubplot = currSubplot + 1;       
        hold on; grid on;
        title("Distance from ground truth");
        plot(results(8,1:numSteps), 'linewidth', 2);
        ylim([0.0 8])
        ylabel('meters', 'fontsize', 14);
        xlabel('Iterations', 'fontsize', 14);
    end
    
    

end
toc;
