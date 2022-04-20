% File: mainPF.m
% Author: Gregor Limstrom <limstrom@umich.edu>
% Purpose: Open data and explore format for 530 project
% Note - right click on mainPF.m to change directory to that in order to
% work properly
% 
% clc;clear; close all;

srcFolderPath = pwd;
addpath PF_helpers
addpath(srcFolderPath)
cd("../");
topLevelPath = pwd;

cd(topLevelPath)

%% Load data from MRCLAM9
%run('data\MRCLAM9\loadMRCLAMdataSet.m')

% numSteps
% useGTOnly
% useLandmarksOnly
% useTrustFactor
% trustFactorTime


% Load data from MRCLAM 1
run('data\MRCLAM1\loadMRCLAMdataSetPF.m')

% Sample data
dt = 0.02;
[Robots, timesteps] = sampleMRCLAMdataSetPF(Robots, 0.02);

% Set field info to landmark GT
global FIELDINFO;
FIELDINFO = Landmark_Groundtruth;
global ROBOT_ESTIMATES;
% x,y,theta, time since last measurement update
ROBOT_ESTIMATES = zeros(5,4);

% Add pose estimate
for i = 1:5
    Robots{i}.Est = zeros(size(Robots{i}.G,1), 4);
    % Initialize robot estimates to initial groundtruth
    ROBOT_ESTIMATES(i,1:3) = Robots{i}.G(1,2:4);
end



% Create data struct to track #lm and #robots seen, # meas > 2 lm
global MEAS_STATS;
MEAS_STATS = zeros(5,3);

%% Particle filter for localization
% Update based on measurements of landmarks and other robots
% Iterate through sampled data

% Robot 1 Solo Start/End
start = 1;
%start = 1;
% Start time
t = Robots{1}.G(start, 1);

% End index and time
end_idx = numSteps;

% Num robots
num_robots = 5;

% Map between barcodes and landmark id
codeDict = containers.Map(Barcodes(:,2), Barcodes(:,1));

% Move 

% Set up system
R = diag([0.1^2, 0.05^2]);
% Cholesky factor
L = chol(R, 'lower');

% State
% create filter parameters for each robot
%
r1_sys = [];
r1_sys.h = @(landmark_x, landmark_y, mu_pred) [...
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2) ;
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));];
r1_sys.Q = 0.1 * eye(3);
r1_sys.R = R;
r1_sys.last_timestep = t;

r2_sys = [];
r2_sys.h = @(landmark_x, landmark_y, mu_pred) [...
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2) ;
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));];
r2_sys.Q = 0.1 * eye(3);
r2_sys.R = R;
r2_sys.last_timestep = t;

r3_sys = [];
r3_sys.h = @(landmark_x, landmark_y, mu_pred) [...
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2) ;
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));];
r3_sys.Q = 0.1 * eye(3);
r3_sys.R = R;
r3_sys.last_timestep = t;

r4_sys = [];
r4_sys.h = @(landmark_x, landmark_y, mu_pred) [...
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2) ;
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));];
r4_sys.Q = 0.1 * eye(3);
r4_sys.R = R;
r4_sys.last_timestep = t;

r5_sys = [];
r5_sys.h = @(landmark_x, landmark_y, mu_pred) [...
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2) ;
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));];
r5_sys.Q = 0.1 * eye(3);
r5_sys.R = R;
r5_sys.last_timestep = t;

% Initialization
%
%

num_particles = 100;

% For robot 1
init1 = [];
init1.n = num_particles;
init1.x = [Robots{1}.G(start,2); 
            Robots{1}.G(start,3); 
            Robots{1}.G(start,4)];
init1.Sigma = 0.1*eye(3);
        
init2 = [];
init2.n = num_particles;
init2.x = [Robots{2}.G(start,2); 
            Robots{2}.G(start,3); 
            Robots{2}.G(start,4)];
init2.Sigma = 0.1*eye(3);

init3 = [];
init3.n = num_particles;
init3.x = [Robots{3}.G(start,2); 
            Robots{3}.G(start,3); 
            Robots{3}.G(start,4)];
init3.Sigma = 0.1*eye(3);

init4 = [];
init4.n = num_particles;
init4.x = [Robots{4}.G(start,2); 
            Robots{4}.G(start,3); 
            Robots{4}.G(start,4)];
init4.Sigma = 0.1*eye(3);

init5 = [];
init5.n = num_particles;
init5.x = [Robots{5}.G(start,2); 
            Robots{5}.G(start,3); 
            Robots{5}.G(start,4)];
init5.Sigma = 0.1*eye(3);


% Initialize individual particle filters
filter_1 = particle_filter(r1_sys, init1);
filter_2 = particle_filter(r2_sys, init2);
filter_3 = particle_filter(r3_sys, init3);
filter_4 = particle_filter(r4_sys, init4);
filter_5 = particle_filter(r5_sys, init5);

filters = [filter_1 filter_2 filter_3 filter_4 filter_5];

robot_num = 5;
% track each robot's start index, since they are desync
measurementIndex = [1 1 1 1 1];

for i = start:end_idx
    if(mod(i,1000) == 0)
        disp("On iteration: ")
        disp(i)
    end
    
    % Iterate through robots
    for robot_num = 1:num_robots
        
        % Update command vector
        command_t = [Robots{robot_num}.O(i,2); Robots{robot_num}.O(i,3)];

        % Find measurements that match time step
        [z, measurementIndex(robot_num)] = getObservations(Robots, robot_num, t, measurementIndex(robot_num), codeDict, useLandmarksOnly);

        % If we have measurement, update with that. Otherwise use odo
        filters(robot_num).motion_update(command_t, t);
        if z(1,1) ~= -1
            filters(robot_num).measurement_update(robot_num, z, useTrustFactor, trustFactorTime);
        end

        wtot = sum(filters(robot_num).p.w);
        if wtot > 0
            
            use_robot_gt = false;
            
            % update state by sum and average particles
            poseMean = zeros(3,1);
            poseMean(1) = sum(filters(robot_num).p.x(1,:)' .* ...
                filters(robot_num).p.w) / wtot;
            poseMean(2) = sum(filters(robot_num).p.x(2,:)' .* ...
                filters(robot_num).p.w) / wtot;
            poseMean(3) = sum(filters(robot_num).p.x(3,:)' .* ...
                filters(robot_num).p.w) / wtot;
            poseMean(3) = wrapToPi(poseMean(3));
            Robots{robot_num}.Est(i,:) = [t poseMean(1) poseMean(2) ...
                poseMean(3)];
            if(useGTOnly)
                ROBOT_ESTIMATES(robot_num,1:3) = Robots{robot_num}.G(i,2:4);
            else
                ROBOT_ESTIMATES(robot_num,1:3) = [poseMean(1) poseMean(2) ...
                    poseMean(3)];
            end
        else
            warning('Total weight is zero or nan!')
            disp(wtot);
            disp(filter.p.w);
            Robots{robot_num}.Est(i,:) = [t nan(1,3)];
        end    
        ROBOT_ESTIMATES(robot_num, 4) = ROBOT_ESTIMATES(robot_num, 4) + 1;
    end
    % Increment timestep
    t = t + 0.02;
end

%% Animate results
%animateMRCLAMdataSet(Robots, Barcodes, Landmark_Groundtruth, timesteps, 0.02, start, end_idx);

%% Plot error
results_1_PF = zeros(8, 30001-1000);
results_2_PF = zeros(8, 30001-1000);
results_3_PF = zeros(8, 30001-1000);
results_4_PF = zeros(8, 30001-1000);
results_5_PF = zeros(8, 30001-1000);

hold on
for i = 1:5
    if i == 1
        results_1_PF(8,:) = square_err(Robots{i}.G(1000:30000,2:4), Robots{i}.Est(1000:30000,2:4))';
    elseif i == 2
        results_2_PF(8,:) = square_err(Robots{i}.G(1000:30000,2:4), Robots{i}.Est(1000:30000,2:4))';
    elseif i == 3
        results_3_PF(8,:) = square_err(Robots{i}.G(1000:30000,2:4), Robots{i}.Est(1000:30000,2:4))';
    elseif i == 4
        results_4_PF(8,:) = square_err(Robots{i}.G(1000:30000,2:4), Robots{i}.Est(1000:30000,2:4))';
    elseif i == 5
        results_5_PF(8,:) = square_err(Robots{i}.G(1000:30000,2:4), Robots{i}.Est(1000:30000,2:4))';
    end
        plot(square_err(Robots{i}.G(1000:30000,2:4), Robots{i}.Est(1000:30000,2:4)));
    ylim([0, 5]);
end
hold off

% Ask other robot location, their estimated accuracy rating (no. landmarks
% in their view)
% % Some chance of failure (packet loss)
% % 
function [z, idx] = getObservations(Robots, robot_num, t, idx, codeDict, useLandmarksOnly)
    global MEAS_STATS;

    % Built vector of landmarks observed at current timestep
    z = zeros(3,1);
    z(1,1) = -1;
    
    lm_only = false;
    
    while(Robots{robot_num}.M(idx, 1) - t < 0.005) && (idx < size(Robots{robot_num}.M, 1))
        barcode = Robots{robot_num}.M(idx, 2);
        landmarkID = 0;
        if (codeDict.isKey(barcode))
            landmarkID = codeDict(barcode);
        else
            disp("Error: Key not found")
        end
        if(useLandmarksOnly)
            if(landmarkID > 5)
                MEAS_STATS(robot_num,1) = MEAS_STATS(robot_num,1) + 1;
                range = Robots{robot_num}.M(idx, 3);
                bearing = Robots{robot_num}.M(idx, 4);
                if uint8(z(3)) == 0
                    z = [range;bearing;landmarkID];
                else
                     % Append
                     newZ = [range;bearing;landmarkID];
                     z = [z newZ];
                end
            end
        else
            if(landmarkID > 5)
                MEAS_STATS(robot_num,1) = MEAS_STATS(robot_num,1) + 1;
            else
                % Measurement is of a fellow bot
                MEAS_STATS(robot_num,2) = MEAS_STATS(robot_num,2) + 1;
                %disp("I see a friend!")
            end
            range = Robots{robot_num}.M(idx, 3);
            bearing = Robots{robot_num}.M(idx, 4);
            if uint8(z(3)) == 0
                z = [range;bearing;landmarkID];
            else
                 % Append
                 newZ = [range;bearing;landmarkID];
                 z = [z newZ];
            end
        end    
        idx = idx + 1;
    end
end

% dt is 0.02
% check err every 0.5 sec
function [err] = square_err(groundtruth, estimate)
    len = size(groundtruth(:,1),1);
    err = zeros(len,1);
    for i = 1:len
        err(i) = sqrt((groundtruth(i,1) - estimate(i,1))^2 + ...
        (groundtruth(i,2) - estimate(i,2))^2); 
    end
end
  

% 
% Draws dataset and pose estimates
%
%
function animateMRCLAMdataSet(Robots, Barcodes, Landmark_Groundtruth, timesteps, sample_time, start_t, end_t)

    % Options %
    start_timestep = start_t;
%     end_timestep = timesteps;
    end_timestep = end_t;
    timesteps_per_frame = 25;
    pause_time_between_frames=0.02; %[s]
    draw_measurements = 0;
    % Options END %

    n_robots = size(Robots, 2);
    n_landmarks = length(Landmark_Groundtruth(:,1));

    % Plots and Figure Setup
    % Robot Colors
    colour(1,:) = [1 0 0];
    colour(2,:) = [0 0.75 0];
    colour(3,:) = [0 0 1];
    colour(4,:) = [1 0.50 0.25];
    colour(5,:) = [1 0.5 1];
    % Estimated Robot Colors
    colour(6,:) = [.67 0 0];
    colour(7,:) = [0 0.5 0];
    colour(8,:) = [0 0 .67];
    colour(9,:) = [.67 0.33 0.17];
    colour(10,:) = [.67 0.33 .67];
    for i=11:11+n_landmarks
        colour(i,:) = [0.3 0.3 0.3]; 
    end

    figHandle = figure('Name','Dataset Groundtruth','Renderer','OpenGL');
    set(gcf,'Position',[1300 1 630 950])
    plotHandles_robot_gt = zeros(n_robots,1);
    plotHandles_robot_est = zeros(n_robots,1);
    plotHandles_landmark_gt = zeros(n_landmarks,1);

    r_robot = 0.165;
    d_robot = 2*r_robot;
    r_landmark = 0.055;
    d_landmark = 2*r_landmark;
    
    % get initial positions for robot groundtruth
    for i = 1:n_robots  
        x=Robots{i}.G(1,2);
        y=Robots{i}.G(1,3);
        z=Robots{i}.G(1,4);
        x1 = d_robot*cos(z) + x;
        y1 = d_robot*sin(z) + y;
        p1 = x - r_robot;
        p2 = y - r_robot;
        plotHandles_robot_gt(i) = rectangle('Position',[p1,p2,d_robot,d_robot],'Curvature',[1,1],...
                  'FaceColor',colour(i,:),'LineWidth',1);
        line([x x1],[y y1],'Color','k');    
        n_measurements(i) = length(Robots{i}.M(:,1));
    end
    
    % get initial positions for robot pose estimates
    for i = 1:n_robots
        x=Robots{i}.Est(1,2);
        y=Robots{i}.Est(1,3);
        z=Robots{i}.Est(1,4);
        x1 = d_robot*cos(z) + x;
        y1 = d_robot*sin(z) + y;
        p1 = x - r_robot;
        p2 = y - r_robot;
        plotHandles_robot_est(i) = rectangle('Position',[p1,p2,d_robot,d_robot],'Curvature',[1,1],...
                  'FaceColor',colour(i + 5,:),'LineWidth',1);
        line([x x1],[y y1],'Color','k');    
    end
    
    % get positions for landmarks
    for i = 1:n_landmarks
        x=Landmark_Groundtruth(i, 2);
        y=Landmark_Groundtruth(i, 3);
        p1 = x - r_landmark;
        p2 = y - r_landmark;
        plotHandles_landmark_gt(i) = rectangle('Position',[p1,p2,d_landmark,d_landmark],'Curvature',[1,1],...
                  'FaceColor',colour(i+10,:),'LineWidth',1);
    end

    axis square;
    axis equal;
    axis([-2 6 -6 7]);
    set(gca,'XTick',(-10:2:10)');

    % Going throuhg data
    measurement_time_index = ones(n_robots,1); % index of last measurement processed
    barcode = 0;
    for i=1:n_robots
        tempIndex=find(Robots{i}.M(:,1)>=start_timestep*sample_time,1,'first');
        if ~isempty(tempIndex)
            measurement_time_index(i) = tempIndex;
        else
            measurement_time_index(i) = n_measurements(i)+1;
        end
    end
    clear tempIndex
    
    % animate robots groundtruth and pose estimates
    pause(5);
    for k=start_timestep:end_timestep
        t = k*sample_time;

        if(mod(k,timesteps_per_frame)==0)
            delete(findobj('Type','line')); 
        end

        for i= 1:n_robots        

            % GT
            x_g(i) = Robots{i}.G(k,2);
            y_g(i) = Robots{i}.G(k,3);
            z_g(i) = Robots{i}.G(k,4);
            % EST
            x_est(i) = Robots{i}.Est(k,2);
            y_est(i) = Robots{i}.Est(k,3);
            z_est(i) = Robots{i}.Est(k,4);
            % Particles
            
            if(mod(k,timesteps_per_frame)==0)
                x1_g = d_robot*cos(z_g(i)) + x_g(i);
                y1_g = d_robot*sin(z_g(i)) + y_g(i);
                p1_g = x_g(i) - r_robot;
                p2_g = y_g(i) - r_robot;
                set(plotHandles_robot_gt(i),'Position',[p1_g,p2_g,d_robot,d_robot]);
                line([x_g(i) x1_g],[y_g(i) y1_g],'Color','k');
                
                x1_est = d_robot*cos(z_est(i)) + x_est(i);
                y1_est = d_robot*sin(z_est(i)) + y_est(i);
                p1_est = x_est(i) - r_robot;
                p2_est = y_est(i) - r_robot;
                set(plotHandles_robot_est(i),'Position',[p1_est,p2_est,d_robot,d_robot]);
                line([x_est(i) x1_est],[y_est(i) y1_est],'Color','k');
            end

            % plot meaurements of robot i 
            if(draw_measurements)
                while(n_measurements(i) >= measurement_time_index(i) && Robots{i}.M(measurement_time_index(i),1) <= t)
                    measure_id = Robots{i}.M(measurement_time_index(i),2);
                    measure_r = Robots{i}.M(measurement_time_index(i),3);
                    measure_b = Robots{i}.M(measurement_time_index(i),4);
                    landmark_index = find(Barcodes(:,2)==measure_id);
                    if(~isempty(landmark_index))
                        x1 = x(i) + measure_r*cos(measure_b + z(i));
                        y1 = y(i) + measure_r*sin(measure_b + z(i));
                        line([x(i) x1],[y(i) y1],'Color',colour(i,:),'LineWidth',1);
                    else
                        robot_index = find(Barcodes(1:5,2)==measure_id);
                        if(~isempty(robot_index))
                            x1 = x(i) + measure_r*cos(measure_b + z(i));
                            y1 = y(i) + measure_r*sin(measure_b + z(i));
                            line([x(i) x1],[y(i) y1],'Color',colour(i,:),'LineWidth',1);
                        end
                    end
                    measurement_time_index(i) = measurement_time_index(i) + 1; 
                end
            end
        end

        % write time
        if(mod(k,timesteps_per_frame)==0)
            delete(findobj('Type','text'));
            texttime = strcat('k= ',num2str(k,'%5d'), '  t= ',num2str(t,'%5.2f'), '[s]');
            text(1.5,6.5,texttime);
            pause(pause_time_between_frames);
        else 
            if(draw_measurements)
                pause(0.01);
            end
        end
    end

    clear Robot x x1 y y1 z r_robot r_landmark p1 p2 max_time measure_id d_robot d_landmark barcode k colour
    clear texttime t measurement_time_index masure_id measure_r measure_b landmark_index robot_index i n_measurements plotHandles* angle

end
