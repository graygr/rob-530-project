% System Initialization
alphas = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2;
beta = 0.02;
gamma = 0.03;
sys = system_initialization(alphas, beta, gamma, sample_time);

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
init2.kappa_g = 0;
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

global ROBOT1 ROBOT2 ROBOT3 ROBOT4 ROBOT5 LANDMARK BAR
% ROBOT1 = Robot1_Groundtruth;
% ROBOT2 = Robot2_Groundtruth;
% ROBOT3 = Robot3_Groundtruth;
% ROBOT4 = Robot4_Groundtruth;
% ROBOT5 = Robot5_Groundtruth;
ROBOT1 = filter1;
ROBOT2 = filter2;
ROBOT3 = filter3;
ROBOT4 = filter4;
ROBOT5 = filter5;
BAR = Barcodes;
LANDMARK = Landmark_Groundtruth;

nsteps = 75000;
for t = 1:nsteps
    % Robot1 Prediction & Correction
    motionCommand1 = Robot1_Odometry(t, 2:3);
    filter1.prediction(motionCommand1);
    idx1 = find(abs(Robot1_Measurement(:, 1)-Robot1_Odometry(t, 1))<0.0001);
    if isempty(idx1)
        filter1.mu = filter1.mu_pred;
        filter1.Sigma = filter1.Sigma_pred;
    else
        observation1 = Robot1_Measurement(idx1, :);
        filter1.correction(observation1);
    end
    Robot1_Correction(t,:) = filter1.mu;
    
    %Robot2 Prediction & Correction
    motionCommand2 = Robot2_Odometry(t, 2:3);
    filter2.prediction(motionCommand2);
    idx2 = find(abs(Robot2_Measurement(:, 1)-Robot2_Odometry(t, 1))<0.0001);
    if isempty(idx2)
        filter2.mu = filter2.mu_pred;
        filter2.Sigma = filter2.Sigma_pred;
    else
        observation2 = Robot2_Measurement(idx2, :);
        filter2.correction(observation2);
    end
    Robot2_Correction(t,:) = filter2.mu;

    %Robot3 Prediction & Correction
    motionCommand3 = Robot3_Odometry(t, 2:3);
    filter3.prediction(motionCommand3);
    idx3 = find(abs(Robot3_Measurement(:, 1)-Robot3_Odometry(t, 1))<0.0001);
    if isempty(idx3)
        filter3.mu = filter3.mu_pred;
        filter3.Sigma = filter3.Sigma_pred;
    else
        observation3 = Robot3_Measurement(idx3, :);
        filter3.correction(observation3);
    end
    Robot3_Correction(t,:) = filter3.mu;

    %Robot4 Prediction & Correction
    motionCommand4 = Robot4_Odometry(t, 2:3);
    filter4.prediction(motionCommand4);
    idx4 = find(abs(Robot4_Measurement(:, 1)-Robot4_Odometry(t, 1))<0.0001);
    if isempty(idx4)
        filter4.mu = filter4.mu_pred;
        filter4.Sigma = filter4.Sigma_pred;
    else
        observation4 = Robot4_Measurement(idx4, :);
        filter4.correction(observation4);
    end
    Robot4_Correction(t,:) = filter4.mu;

    %Robot5 Prediction & Correction
    motionCommand5 = Robot5_Odometry(t, 2:3);
    filter5.prediction(motionCommand5);
    idx5 = find(abs(Robot5_Measurement(:, 1)-Robot5_Odometry(t, 1))<0.0001);
    if isempty(idx5)
        filter5.mu = filter5.mu_pred;
        filter5.Sigma = filter5.Sigma_pred;
    else
        observation5 = Robot5_Measurement(idx5, :);
        filter5.correction(observation5);
    end
    Robot5_Correction(t,:) = filter5.mu;
end
error1 = sqrt((Robot1_Correction(1:nsteps,1)-Robot1_Groundtruth(1:nsteps,2)).^2+(Robot1_Correction(1:nsteps,2)-Robot1_Groundtruth(1:nsteps,3)).^2);
error2 = sqrt((Robot2_Correction(1:nsteps,1)-Robot2_Groundtruth(1:nsteps,2)).^2+(Robot2_Correction(1:nsteps,2)-Robot2_Groundtruth(1:nsteps,3)).^2);
error3 = sqrt((Robot3_Correction(1:nsteps,1)-Robot3_Groundtruth(1:nsteps,2)).^2+(Robot3_Correction(1:nsteps,2)-Robot3_Groundtruth(1:nsteps,3)).^2);
error4 = sqrt((Robot4_Correction(1:nsteps,1)-Robot4_Groundtruth(1:nsteps,2)).^2+(Robot4_Correction(1:nsteps,2)-Robot4_Groundtruth(1:nsteps,3)).^2);
error5 = sqrt((Robot5_Correction(1:nsteps,1)-Robot5_Groundtruth(1:nsteps,2)).^2+(Robot5_Correction(1:nsteps,2)-Robot5_Groundtruth(1:nsteps,3)).^2);
distance_MSE = [mean(error1); mean(error2); mean(error3); mean(error4); mean(error5)];

angle_error1 = abs(wrapToPi(Robot1_Correction(1:nsteps,3)-Robot1_Groundtruth(1:nsteps,4)));
angle_error2 = abs(wrapToPi(Robot2_Correction(1:nsteps,3)-Robot2_Groundtruth(1:nsteps,4)));
angle_error3 = abs(wrapToPi(Robot3_Correction(1:nsteps,3)-Robot3_Groundtruth(1:nsteps,4)));
angle_error4 = abs(wrapToPi(Robot4_Correction(1:nsteps,3)-Robot4_Groundtruth(1:nsteps,4)));
angle_error5 = abs(wrapToPi(Robot5_Correction(1:nsteps,3)-Robot5_Groundtruth(1:nsteps,4)));
angle_MSE = [mean(angle_error1); mean(angle_error2); mean(angle_error3); mean(angle_error4); mean(angle_error5)];

figure(1);
plot(0:0.02:(nsteps-1)*0.02, error1, 0:0.02:(nsteps-1)*0.02, error2, 0:0.02:(nsteps-1)*0.02, error3, 0:0.02:(nsteps-1)*0.02, error4, 0:0.02:(nsteps-1)*0.02, error5, LineWidth=1.5);
xlabel('Time(s)');
ylabel('Distance(m)');
legend('Robot1', 'Robot2', 'Robot3', 'Robot4', 'Robot5');
title('Distance Error(UKF using landmark and robots groundtruth)');
axis([0 1500 0 3]);

figure(2);
plot(0:0.02:(nsteps-1)*0.02, angle_error1, 0:0.02:(nsteps-1)*0.02, angle_error2, 0:0.02:(nsteps-1)*0.02, angle_error3, 0:0.02:(nsteps-1)*0.02, angle_error4, 0:0.02:(nsteps-1)*0.02, angle_error5, LineWidth=1.5);
xlabel('Time(s)');
ylabel('Angle(rad)');
legend('Robot1', 'Robot2', 'Robot3', 'Robot4', 'Robot5');
title('Angle Error(UKF using landmark and robots groundtruth)');
