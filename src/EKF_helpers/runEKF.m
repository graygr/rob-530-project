
% plotSigmaEllipses = false;

disp("Running EKF with robots:");
disp(robotsToRun);
tic;
waitbar_h = waitbar(0,'Waitbar EKF');


%Assume for now that all robots will use same alphas and beta
% Motion noise (in odometry space, see Table 5.5, p.134 in book).

deltaT = 0.02; 
timeTrustNumSteps = trustFactorTime/deltaT;

systems = {};
filters = {};
for i=1:length(robotsToRun)
    id = robotsToRun(i);
    %Get the initialStateMean and Cov for each robot, variables will use the
    %name and tack on their id
    eval(['initialStateMean' num2str(id) ' = Robot' num2str(id) '_Groundtruth(1,2:end)'';'])
    eval(['initialStateCov' num2str(id) ' = eye(3);'])
    sys = system_initializationEKF(alphas,  beta,  [0 0 0 0], deltaT);
    systems{i} = sys;
    eval(['filters{i}=filter_initialization(sys,  initialStateMean' num2str(id) ',  initialStateCov' num2str(id) ', "EKF", deltaT);'])
    
    eval(['observationsUsedatIndex' num2str(id) ' = [];'])
    eval(['robotPose' num2str(id) ' = zeros(numSteps,3);'])
    eval(['robotSigmas' num2str(id) ' = {};'])
    eval(['lastvalidMeasurementIndex' num2str(id) ' = 1;'])
%     eval(['rejectedUpdates' num2str(id) ' = 0;'])
    eval(['countBadObservationIDMismatch' num2str(id) ' = 0;'])  
    eval(['fixedBadObservationIDMismatch' num2str(id) ' = 0;'])    
    eval(['numLandmarksObsUsed' num2str(id) ' = 0;'])   
    eval(['numRobotObsUsed' num2str(id) ' = 0;'])   
    eval(['numUpdatesUsed' num2str(id) ' = 0;'])    
    eval(['results' num2str(id) 'EKF = zeros(8,numSteps);'])
    
    eval(['tStepLastUpdate' num2str(id) ' = 0;'])   
    eval(['rejectedDueToTrustTStep' num2str(id) ' = 0;'])    
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

landmarkDistanceThresholdSquared = landmarkDistanceThreshold^2;
likelyLandmarkDistanceThresholdSquared = likelyLandmarkDistanceThreshold^2; 

observationsDiff = [];
observationsDiffIndex = 1;
%
lastPerc = 0;
for t = 1:numSteps
    
    perc = t/numSteps;
    if (abs(perc-lastPerc)>.01)
        lastPerc = perc;
        waitbar(perc,waitbar_h,sprintf('%f%% along EKF...',perc*100))
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
        myPose = filters{i}.mu;
        observations = [];
        obsCol = 1;
        if ~isempty(measurements) &&  useObservationsToCorrect    
            for j=1:size(measurements,1)
                if ~isKey(subjectNumToIDMAP,measurements(j,1))
                    continue
                end
                idObserved = subjectNumToIDMAP(measurements(j,1));
                measuredBearing = measurements(j,3);
                measuredRange = measurements(j,2);
                
    
                if (idObserved < 6) && ~useLandmarksOnly
                    if useGTForObservedRobots
        %                 disp(['observed robot: ',num2str(id)])
                        eval(['gt = Robot' num2str(idObserved) '_Groundtruth(' num2str(t) ',1:4);'])
                        if (gt(1) ~= currTime)
                            error("GT time of observed robot does not match curr time.");
                        end
                        observations(1, obsCol) = measuredBearing; %get bearing from measurment
                        observations(2, obsCol) = measuredRange; %get range from measurment
                        observations(3, obsCol) = gt(2); %get x global pos of observed id
                        observations(4, obsCol) = gt(3); %get y global pos of observed id
                        observations(5, obsCol) = idObserved; %observed id
                        observations(6, obsCol) = measurements(j,1); %barcode id
                        obsCol = obsCol +1;
                        eval(['numRobotObsUsed' num2str(id) ' =numRobotObsUsed' num2str(id) ' +1;'])   
                    elseif useEstimateForObservedRobots
                        
                        eval(['tStepLastUpdate = tStepLastUpdate' num2str(idObserved) ';']) 
                        if t-tStepLastUpdate > timeTrustNumSteps
                            % we do not trust the observed robots pose
                            eval(['rejectedDueToTrustTStep' num2str(id) ' =rejectedDueToTrustTStep' num2str(id) ' +1;'])    
                            continue;
                        end
                        eval(['obsRobotEstimate = robotPose' num2str(id) '(t-1,:);'])
                        observations(1, obsCol) = measuredBearing; %get bearing from measurment
                        observations(2, obsCol) = measuredRange; %get range from measurment
                        observations(3, obsCol) = obsRobotEstimate(1); %get x global pos of observed id
                        observations(4, obsCol) = obsRobotEstimate(2); %get y global pos of observed id
                        observations(5, obsCol) = idObserved; %observed id
                        observations(6, obsCol) = measurements(j,1); %barcode id
                        obsCol = obsCol +1;
                        eval(['numRobotObsUsed' num2str(id) ' =numRobotObsUsed' num2str(id) ' +1;'])   
                        
                    else
                        error("Not setup for this method of observing other robots");
                    end

                elseif (idObserved >= 6)
    %                 disp(['observed landmark: ',num2str(id)])
                    % The measurements may have the wrong landmark! Ignore
                    % the observation if the perceived landmark range and
                    % bearing is more than landmarkDistanceThreshold 
                    
                    x1 = myPose(1) + measuredRange*cos(measuredBearing + myPose(3));
                    y1 = myPose(2) + measuredRange*sin(measuredBearing + myPose(3));
                    landmarkX = landmarkIDToXMAP(idObserved);
                    landmarkY = landmarkIDToYMAP(idObserved);
                    
                    distanceSquared = (landmarkX-x1)^2+(landmarkY-y1)^2;
                    if distanceSquared > landmarkDistanceThresholdSquared
%                         disp("Warning, observed landmark id position does not agree with range/bearing measurement");
                        eval(['countBadObservationIDMismatch' num2str(id) ...
                            ' = countBadObservationIDMismatch' num2str(id) ' + 1;'])
                        % see if there are any other likely landmarks, 
                        foundLikely = false;
                        bestLikelyDistance = Inf;
                        bestLikelyLandmarkId = 0;
                        bestLikelyLandmarkX = 0;
                        bestLikelyLandmarkY = 0;
                        for findLikelyLandmark=1:n_landmarks
                            currLandmarkId = Landmark_Groundtruth(findLikelyLandmark,1);
                            currLandmarkX = Landmark_Groundtruth(findLikelyLandmark,2);
                            currLandmarkY = Landmark_Groundtruth(findLikelyLandmark,3);
                            
                            currDistanceSquared = (currLandmarkX-x1)^2+(currLandmarkY-y1)^2;
                            if currDistanceSquared < bestLikelyDistance && currDistanceSquared < likelyLandmarkDistanceThresholdSquared
                                foundLikely = true;
                                bestLikelyDistance = currDistanceSquared;
                                bestLikelyLandmarkId = currLandmarkId;
                                bestLikelyLandmarkX = currLandmarkX;
                                bestLikelyLandmarkY = currLandmarkY;
                            end
                        end
                        
                        if foundLikely == true
%                             disp(['Found likely landmark to be ' num2str(bestLikelyLandmarkId) 'instead of ' num2str(idObserved)]);
                            landmarkX = bestLikelyLandmarkX;
                            landmarkY = bestLikelyLandmarkY;
                            idObserved = bestLikelyLandmarkId;
                            likelyLandmarbarcodeId = Barcodes(find(Barcodes(:,1)==bestLikelyLandmarkId),2);
                            measurements(j,1) = likelyLandmarbarcodeId;
                            eval(['fixedBadObservationIDMismatch' num2str(id) ...
                                ' = fixedBadObservationIDMismatch' num2str(id) ' + 1;'])
                        else
                            continue;
                        end
                    end
                    
                    observations(1, obsCol) = measuredBearing; %get bearing from measurment
                    observations(2, obsCol) = measuredRange; %get range from measurment
                    observations(3, obsCol) = landmarkX; %get x global pos of observed id
                    observations(4, obsCol) = landmarkY; %get y global pos of observed id
                    observations(5, obsCol) = idObserved; %observed id
                    observations(6, obsCol) = measurements(j,1); %barcode id
                    obsCol = obsCol +1;
                    eval(['numLandmarksObsUsed' num2str(id) ' =numLandmarksObsUsed' num2str(id) ' +1;'])   
                end
%                 %Debugging observation issues
%                 eval(['mygt = Robot' num2str(id) '_Groundtruth(' num2str(t) ',1:4);'])
%                 gtx = mygt(2);
%                 gty = mygt(3);
%                 gtt = mygt(4);
%                 obsx = observations(3, obsCol-1);
%                 obsy = observations(4, obsCol-1);
%                 deltaX = (obsx-gtx);
%                 deltaY = (obsy-gty);
%                 actualBearing = wrapToPi(atan2(deltaY,deltaX)-gtt);
%                 actualRange = sqrt(deltaX^2+deltaY^2);
%                 diffBearing = wrapToPi(observations(1, obsCol-1) - actualBearing);
%                 diffRange = observations(2, obsCol-1) - actualRange;
%                 mat = [t measuredBearing actualBearing diffBearing ...
%                     measuredRange actualRange diffRange ...
%                     deltaX deltaY ...
%                     gtx gty gtt obsx obsy ];
%                 observationsDiff(observationsDiffIndex, 1:length(mat)) =mat;
%                 observationsDiffIndex = observationsDiffIndex + 1;
            end

        end
        %There must be at least two observations for proper dimensions in correction 
        observationsAvailable =  size(observations,2)>=2;
    
       filters{i}.prediction(noisyMotionCommand);
%        muPrev = filters{i}.mu_pred;
%        sigmaPrev = filters{i}.Sigma_pred;
       
       if observationsAvailable && useObservationsToCorrect
           eval(['observationsUsedatIndex' num2str(id) ' = [observationsUsedatIndex' num2str(id) ' t];'])
           filters{i}.correction(observations);
           eval(['numUpdatesUsed' num2str(id) ' =numUpdatesUsed' num2str(id) ' +1;']) 
           eval(['tStepLastUpdate' num2str(id) '= t;'])
       else
           filters{i}.setPredictionAsCurrent();
       end
%        muAfter = filters{i}.mu;
       
%        distance = sqrt((muPrev(1)-muAfter(1))^2+(muPrev(2)-muAfter(2))^2);
%        if distance >= rejectDistanceThreshold
% %            disp("Too large of jump, using mu/sigma before update");
%            eval(['rejectedUpdates' num2str(id) ' = rejectedUpdates' num2str(id) '+1;'])
%            filters{i}.mu = muPrev;
%            filters{i}.Sigma = sigmaPrev;
%        end
       
       eval(['robotPose' num2str(id) '(t,:) = filters{i}.mu(1:3)'';'])
       eval(['robotSigmas' num2str(id) '{t} = filters{i}.Sigma;'])
       eval(['GTMu = Robot' num2str(id) '_Groundtruth(t,2:4)'';'])
       eval(['results' num2str(id) 'EKF(:,t) = mahalanobis(filters{i}.mu,filters{i}.Sigma,GTMu);'])
    end
end
toc;
close(waitbar_h);
%% Plot
disp("Plotting results");
tic;

numSteps = t;
for i=1:length(robotsToRun)
    id = robotsToRun(i);
%     eval(['numRejected = rejectedUpdates' num2str(id) ';'])
    disp(['Robot ' num2str(id) ':']);
    
    robotEstimatedXdata = [];
    robotEstimatedYdata = [];
    robotEstimatedTdata = [];
    robotSigmas = {};
    sigmaDiags = [];
    robotGTXdata = [];
    robotGTYdata = [];
    robotGTTdata = [];
    observationsUsedatIndex = [];
    robotVCommands = [];
    robotWCommands = [];
    eval(['robotEstimatedXdata = robotPose' num2str(id) '(1:numSteps,1);'])
    eval(['robotEstimatedYdata = robotPose' num2str(id) '(1:numSteps,2);'])
    eval(['robotEstimatedTdata = robotPose' num2str(id) '(1:numSteps,3);'])
    eval(['robotSigmas = robotSigmas' num2str(id) ';'])
    eval(['robotGTXdata = Robot' num2str(id) '_Groundtruth(1:numSteps,2);'])
    eval(['robotGTYdata = Robot' num2str(id) '_Groundtruth(1:numSteps,3);'])
    eval(['robotGTTdata = Robot' num2str(id) '_Groundtruth(1:numSteps,4);'])
    eval(['observationsUsedatIndex = observationsUsedatIndex' num2str(id) ';'])
    eval(['robotVCommands = Robot' num2str(id) '_Odometry(1:numSteps,2);'])
    eval(['robotWCommands = Robot' num2str(id) '_Odometry(1:numSteps,3);'])
    eval(['countBadObservationIDMismatch = countBadObservationIDMismatch' num2str(id) ';'])
    eval(['fixedBadObservationIDMismatch = fixedBadObservationIDMismatch' num2str(id) ';'])
    
    
    for j=1:length(robotSigmas)
        sigmaDiags(:,j) = diag(robotSigmas{j});
    end
    

    figure('units','normalized','outerposition',[0 0 1 1])
    if useObservationsToCorrect
        if useLandmarksOnly
            msg = ['Robot: ' num2str(id) ' EKF with prediction and correction from landmarks only'];
            sgtitle(msg);
            if i == 1
                disp('EKF with prediction and correction from landmarks only');
            end
            
        elseif useGTForObservedRobots && ~useLandmarksOnly
            msg = ['Robot: ' num2str(id) ' EKF with prediction and correction from gt robots and landmarks'];
            sgtitle(msg);
            if i == 1
                disp('EKF with prediction and correction from gt robots and landmarks');
            end
        else
            msg = ['Robot: ' num2str(id) ' EKF with prediction and correction from est'];
            sgtitle(msg);
            if i == 1
                disp('EKF with prediction and correction from est');
            end
        end
        
    else
        msg = ['Robot: ' num2str(id) ' EKF with prediction only'];
        sgtitle(msg);
        if i == 1
            disp('EKF with prediction only');
        end
    end
%     disp([num2str(numRejected) ' rejected updates.']);
    if countBadObservationIDMismatch ~= 0
        disp([num2str(countBadObservationIDMismatch) ' bad landmark id observations.']);
        disp([num2str(fixedBadObservationIDMismatch) ' fixed landmark id observations.']);
    end
    
    rows = 2;
    cols = 2;
    currSubplot = 1;
    
    subplot(rows,cols,currSubplot);
    currSubplot = currSubplot + 1;
    title("Robot x,y over time");
    hold on;
    plot(robotEstimatedXdata,robotEstimatedYdata,'r.-');
    plot(robotGTXdata,robotGTYdata,'b.-');
%     if plotSigmaEllipses
%         for j=1:numSteps
%             eval(['sigma=robotSigmas' num2str(id) '{j};'])
%             draw_ellipse([robotEstimatedXdata(j);robotEstimatedYdata(j)], sigma(1:2,1:2),1);
%         end
%     end
    legend('Estimated pose','GroundTruth');

    subplot(rows,cols,currSubplot);
    currSubplot = currSubplot + 1;
    title("theta data vs time");
    hold on;
    plot(1:numSteps,robotEstimatedTdata,'r.-');
    plot(1:numSteps,robotGTTdata,'b.-');
    if plotObservationLines(3) 
        for j=1:length(observationsUsedatIndex)
            xline(observationsUsedatIndex(j));
        end
    end
    
    subplot(rows,cols,currSubplot);
    currSubplot = currSubplot + 1;
    title("Xdata vs time");
    hold on;
    plot(1:numSteps,robotEstimatedXdata,'r.-');
    plot(1:numSteps,robotGTXdata,'b.-');
    if plotObservationLines(1) 
        for j=1:length(observationsUsedatIndex)
            xline(observationsUsedatIndex(j));
        end
    end
    
    subplot(rows,cols,currSubplot);
    currSubplot = currSubplot + 1;
    title("Ydata vs time");
    hold on;
    plot(1:numSteps,robotEstimatedYdata,'r.-');
    plot(1:numSteps,robotGTYdata,'b.-');
    if plotObservationLines(2) 
        for j=1:length(observationsUsedatIndex)
            xline(observationsUsedatIndex(j));
        end
    end
    
%     subplot(rows,cols,currSubplot);
%     currSubplot = currSubplot + 1;
%     title("V command vs time");
%     hold on;
%     plot(1:numSteps,robotVCommands,'b.-');
%     
%     subplot(rows,cols,currSubplot);
%     currSubplot = currSubplot + 1;
%     title("W command vs time");
%     hold on;
%     plot(1:numSteps,robotWCommands,'b.-');
    

    eval(['results = results' num2str(id) 'EKF;'])
    distanceRMSE = sqrt(sum(results(8,1:numSteps).^2)/numSteps);
    stdDeviation = std(results(8,1:numSteps));
    disp(['distance RMSE [' num2str(distanceRMSE) '] std deviation [' num2str(stdDeviation) '].']);
        
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

%%
figure();
hold on;
numSteps = t;
for i=1:length(robotsToRun)
    id = robotsToRun(i);
    eval(['rejectedDueToTrustTStep' num2str(id) ''])
    eval(['results = results' num2str(id) 'EKF;'])
    plot(1:numSteps,results(8,1:numSteps));   
    eval(['numLandmarksObsUsed' num2str(id) ''])   
    eval(['numRobotObsUsed' num2str(id) ' '])   
    eval(['numUpdatesUsed' num2str(id) ' '])    
end

if useLandmarksOnly == true
    title("EKF - Landmark Only");
elseif useGTForObservedRobots == true
    title("EKF - Use Robot GT Measurement");
elseif useEstimateForObservedRobots == true
    title("EKF - Use Robot Est w/ 20 sec trust barrier");
end
    
ylim([0 9]);
xlabel("Timestep");
ylabel("Distance Error");
legend("Robot 1","Robot 2","Robot 3","Robot 4","Robot 5");