% System Initialization
alphas = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2;
beta = 0.02;
gamma = 0.03;
sys = system_initialization_UKF(alphas, beta, gamma, sample_time);

%Robots' State Initialization
initialStateMean1 = [3.5732, -3.3328, 2.3408]';
initialStateMean2 = [0.6238, -1.4323, 1.3464]';
initialStateMean3 = [4.3828, 2.4628, -2.3488]';
initialStateMean4 = [0.9918, 2.1241, -0.4336]';
initialStateMean5 = [2.5142, -1.6148, 1.0114]';
initialStateCov = eye(3);
init1.mu = initialStateMean1;
init1.Sigma = initialStateCov;
init1.kappa_g = 1;
init2.mu = initialStateMean2;
init2.Sigma = initialStateCov;
init2.kappa_g = 1;
init3.mu = initialStateMean3;
init3.Sigma = initialStateCov;
init3.kappa_g = 1;
init4.mu = initialStateMean4;
init4.Sigma = initialStateCov;
init4.kappa_g = 1;
init5.mu = initialStateMean5;
init5.Sigma = initialStateCov;
init5.kappa_g = 1;

Robot1_Correction = [];
Robot2_Correction = [];
Robot3_Correction = [];
Robot4_Correction = [];
Robot5_Correction = [];
filter1 = UKF(sys, init1);
filter2 = UKF(sys, init2);
filter3 = UKF(sys, init3);
filter4 = UKF(sys, init4);
filter5 = UKF(sys, init5);
results1UKF = [];
results2UKF = [];
results3UKF = [];
results4UKF = [];
results5UKF = [];

% useGTOnly = false;
% useLandmarksOnly = false;
% useTrustFactor = true;
% trustFactorTime = 20;
% numSteps = 30000;

global ROBOT1 ROBOT2 ROBOT3 ROBOT4 ROBOT5 LANDMARK BAR
if useGTOnly == true
    ROBOT1 = Robot1_Groundtruth;
    ROBOT2 = Robot2_Groundtruth;
    ROBOT3 = Robot3_Groundtruth;
    ROBOT4 = Robot4_Groundtruth;
    ROBOT5 = Robot5_Groundtruth;
else
    ROBOT1 = filter1;
    ROBOT2 = filter2;
    ROBOT3 = filter3;
    ROBOT4 = filter4;
    ROBOT5 = filter5;
end
BAR = Barcodes;
LANDMARK = Landmark_Groundtruth;

lastPerc = 0;
for t = 1:numSteps
    perc = t/numSteps;
    if (abs(perc-lastPerc)>.01)
        lastPerc = perc;
        waitbar(perc,waitbar_h,sprintf('%f%% along UKF...',perc*100))
    end
    
    % Robot1 Prediction & Correction
    motionCommand1 = Robot1_Odometry(t, 2:3);
    filter1.prediction(motionCommand1);
    idx1 = find(abs(Robot1_Measurement(:, 1)-Robot1_Odometry(t, 1))<0.0001);
    if length(idx1)<1
        filter1.mu = filter1.mu_pred;
        filter1.Sigma = filter1.Sigma_pred;
        filter1.time = filter1.time + 1;
    else
        observation1 = Robot1_Measurement(idx1, :);
        filter1.correction(observation1, useGTOnly, useLandmarksOnly, useTrustFactor);
    end
    if filter1.time > trustFactorTime / sample_time
        filter1.trust = false;
    end
    Robot1_Correction(t,:) = filter1.mu;
    results1UKF(:, t) = mahalanobis(filter1.mu, filter1.Sigma, Robot1_Groundtruth(t, 2:4)');
    
    %Robot2 Prediction & Correction
    motionCommand2 = Robot2_Odometry(t, 2:3);
    filter2.prediction(motionCommand2);
    idx2 = find(abs(Robot2_Measurement(:, 1)-Robot2_Odometry(t, 1))<0.0001);
    if length(idx2)<1
        filter2.mu = filter2.mu_pred;
        filter2.Sigma = filter2.Sigma_pred;
        filter2.time = filter2.time + 1;
    else
        observation2 = Robot2_Measurement(idx2, :);
        filter2.correction(observation2, useGTOnly, useLandmarksOnly, useTrustFactor);
    end
    if filter2.time > trustFactorTime / sample_time
        filter2.trust = false;
    end
    Robot2_Correction(t,:) = filter2.mu;
    results2UKF(:, t) = mahalanobis(filter2.mu, filter2.Sigma, Robot2_Groundtruth(t, 2:4)');

    %Robot3 Prediction & Correction
    motionCommand3 = Robot3_Odometry(t, 2:3);
    filter3.prediction(motionCommand3);
    idx3 = find(abs(Robot3_Measurement(:, 1)-Robot3_Odometry(t, 1))<0.0001);
    if length(idx3)<1
        filter3.mu = filter3.mu_pred;
        filter3.Sigma = filter3.Sigma_pred;
        filter3.time = filter3.time + 1;
    else
        observation3 = Robot3_Measurement(idx3, :);
        filter3.correction(observation3, useGTOnly, useLandmarksOnly, useTrustFactor);
    end
    if filter3.time > trustFactorTime / sample_time
        filter3.trust = false;
    end
    Robot3_Correction(t,:) = filter3.mu;
    results3UKF(:, t) = mahalanobis(filter3.mu, filter3.Sigma, Robot3_Groundtruth(t, 2:4)');

    %Robot4 Prediction & Correction
    motionCommand4 = Robot4_Odometry(t, 2:3);
    filter4.prediction(motionCommand4);
    idx4 = find(abs(Robot4_Measurement(:, 1)-Robot4_Odometry(t, 1))<0.0001);
    if length(idx4)<1
        filter4.mu = filter4.mu_pred;
        filter4.Sigma = filter4.Sigma_pred;
        filter4.time = filter4.time + 1;
    else
        observation4 = Robot4_Measurement(idx4, :);
        filter4.correction(observation4, useGTOnly, useLandmarksOnly, useTrustFactor);
    end
    if filter4.time > trustFactorTime / sample_time
        filter4.trust = false;
    end
    Robot4_Correction(t,:) = filter4.mu;
    results4UKF(:, t) = mahalanobis(filter4.mu, filter4.Sigma, Robot4_Groundtruth(t, 2:4)');

    %Robot5 Prediction & Correction
    motionCommand5 = Robot5_Odometry(t, 2:3);
    filter5.prediction(motionCommand5);
    idx5 = find(abs(Robot5_Measurement(:, 1)-Robot5_Odometry(t, 1))<0.0001);
    if length(idx5)<1
        filter5.mu = filter5.mu_pred;
        filter5.Sigma = filter5.Sigma_pred;
        filter5.time = filter5.time + 1;
    else
        observation5 = Robot5_Measurement(idx5, :);
        filter5.correction(observation5, useGTOnly, useLandmarksOnly, useTrustFactor);
    end
    if filter5.time > trustFactorTime / sample_time
        filter5.trust = false;
    end
    Robot5_Correction(t,:) = filter5.mu;
    results5UKF(:, t) = mahalanobis(filter5.mu, filter5.Sigma, Robot5_Groundtruth(t, 2:4)');
end
close(waitbar_h);

% error1 = sqrt((Robot1_Correction(1:numSteps,1)-Robot1_Groundtruth(1:numSteps,2)).^2+(Robot1_Correction(1:numSteps,2)-Robot1_Groundtruth(1:numSteps,3)).^2);
% error2 = sqrt((Robot2_Correction(1:numSteps,1)-Robot2_Groundtruth(1:numSteps,2)).^2+(Robot2_Correction(1:numSteps,2)-Robot2_Groundtruth(1:numSteps,3)).^2);
% error3 = sqrt((Robot3_Correction(1:numSteps,1)-Robot3_Groundtruth(1:numSteps,2)).^2+(Robot3_Correction(1:numSteps,2)-Robot3_Groundtruth(1:numSteps,3)).^2);
% error4 = sqrt((Robot4_Correction(1:numSteps,1)-Robot4_Groundtruth(1:numSteps,2)).^2+(Robot4_Correction(1:numSteps,2)-Robot4_Groundtruth(1:numSteps,3)).^2);
% error5 = sqrt((Robot5_Correction(1:numSteps,1)-Robot5_Groundtruth(1:numSteps,2)).^2+(Robot5_Correction(1:numSteps,2)-Robot5_Groundtruth(1:numSteps,3)).^2);
% distance_MSE = [mean(error1); mean(error2); mean(error3); mean(error4); mean(error5)];
% 
% angle_error1 = abs(wrapToPi(Robot1_Correction(1:numSteps,3)-Robot1_Groundtruth(1:numSteps,4)));
% angle_error2 = abs(wrapToPi(Robot2_Correction(1:numSteps,3)-Robot2_Groundtruth(1:numSteps,4)));
% angle_error3 = abs(wrapToPi(Robot3_Correction(1:numSteps,3)-Robot3_Groundtruth(1:numSteps,4)));
% angle_error4 = abs(wrapToPi(Robot4_Correction(1:numSteps,3)-Robot4_Groundtruth(1:numSteps,4)));
% angle_error5 = abs(wrapToPi(Robot5_Correction(1:numSteps,3)-Robot5_Groundtruth(1:numSteps,4)));
% angle_MSE = [mean(angle_error1); mean(angle_error2); mean(angle_error3); mean(angle_error4); mean(angle_error5)];
% 
% figure(1);
% plot(0:0.02:(numSteps-1)*0.02, error1, 0:0.02:(numSteps-1)*0.02, error2, 0:0.02:(numSteps-1)*0.02, error3, 0:0.02:(numSteps-1)*0.02, error4, 0:0.02:(numSteps-1)*0.02, error5, 'LineWidth',1.5);
% xlabel('Time(s)');
% ylabel('Distance(m)');
% legend('Robot1', 'Robot2', 'Robot3', 'Robot4', 'Robot5');
% title('Distance Error(UKF)');
% 
% figure(2);
% plot(0:0.02:(numSteps-1)*0.02, angle_error1, 0:0.02:(numSteps-1)*0.02, angle_error2, 0:0.02:(numSteps-1)*0.02, angle_error3, 0:0.02:(numSteps-1)*0.02, angle_error4, 0:0.02:(numSteps-1)*0.02, angle_error5, 'LineWidth',1.5);
% xlabel('Time(s)');
% ylabel('Angle(rad)');
% legend('Robot1', 'Robot2', 'Robot3', 'Robot4', 'Robot5');
% title('Angle Error(UKF)');
