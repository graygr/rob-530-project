close all;

% robotsToRun = [1 2 3 4 5];
robotsToRun = [3];
numSteps = length(Robot1_Groundtruth)/8; %number of steps from dataset to run
useGTForObservedRobots = true;
useObservationsToCorrect = true;
%options if you want to show the time steps when observations were used in the [x y theta] plots
plotObservationLines = [false false false]; 

disp("Running EKF with robots:");
disp(robotsToRun);
tic;

%Assume for now that all robots will use same alphas and beta
% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas
% Standard deviation of Gaussian sensor noise (independent of distance)
beta = deg2rad(5);

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
    eval(['lastvalidMeasurementIndex' num2str(id) ' = 1;'])
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
for t = 1:numSteps
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
        %                      yglobalpos1, yglobalpos2,... ]
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

       filters{i}.prediction(noisyMotionCommand);
       if observationsAvailable && useObservationsToCorrect
           eval(['observationsUsedatIndex' num2str(id) ' = [observationsUsedatIndex' num2str(id) ' t];'])
           filters{i}.correction(observations);
       else
           filters{i}.setPredictionAsCurrent();
       end
       
        eval(['robotPose' num2str(id) '(t,:) = filters{i}.mu(1:3)'';'])
    end
    
end
toc;

%% Plot
disp("Plotting results");
tic;
close all;
for i=1:length(robotsToRun)
    id = robotsToRun(i);
    
    
    figure();
    if useObservationsToCorrect
        sgtitle(['Robot: ' num2str(id) ' EKF with prediction and correction where black lines shown']);
    else
        sgtitle(['Robot: ' num2str(id) ' EKF with prediction only, no corrections']);
    end
    
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
    legend('Estimated pose','GroundTruth');

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
end
toc;
