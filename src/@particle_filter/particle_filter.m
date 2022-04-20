classdef particle_filter < handle
    % Particle filter class for state estimation of a nonlinear system
    % The implementation follows the Sample Importance Resampling (SIR)
    % filter a.k.a bootstrap filter.
    %
    %   Template
    %   Author: Maani Ghaffari Jadidi
    %   Date:   01/22/2019
    
    %   Modifications
    %   Author: Gregor Limstrom
    %   Date:   4/20/2022
    %   Contact:limstrom@umich.edu
    
    properties
        f;              % process model
        h;              % measurement model
        x;              % state vector
        mu;
        Sigma;          % state covariance
        Q;              % input noise covariance
        LQ;             % Cholesky factor of Q
        R;              % measurement noise covariance
        p;              % particles
        n;              % number of particles
        Neff;           % effective number of particles
        last_timestep;
    end
    
    methods
        function obj = particle_filter(system, init)
            % particle_filter construct an instance of this class
            %
            %   Inputs:
            %       system          - system and noise models
            %       init            - initialization parameters
            
            obj.Q = system.Q;
            obj.LQ = chol(obj.Q, 'lower');
            obj.h = system.h;
            obj.R = system.R;
            obj.n = init.n;
            obj.last_timestep = system.last_timestep;
            
            % initialize particles
            obj.p = [];
            wu = 1/obj.n; % uniform weights
            L_init = chol(init.Sigma, 'lower');
            for i = 1:obj.n
                obj.p.x(:,i) = L_init * randn(size(init.x,1),1) + init.x;
                obj.p.w(i,1) = wu;
            end
        end
        
        % If we don't have any new measurements, just update based on odo
        function motion_update(obj, control, t)
            % Motion Model:
            % State: [x,y,theta]
            % Control: [timestamp, u,v]
            % 
            
            dt = 0.02;
            
            % For each particle
            for i = 1:obj.n
                % sample noise
                
                % If there is no movement command, do not propogate the
                % particles
                if(control(1) == 0)
                    v_cmd = 0;
                else
                    v_cmd = 0.03 + abs(0.5*control(1));
                end
                if(control(2) == 0)
                    w_cmd = 0.1;
                else
                    w_cmd = 0.07 + abs(0.3*control(2));
                end
                v = normrnd(control(1), v_cmd); % trans
                % Max rot is 0.57
                w = normrnd(control(2), w_cmd); % rot
                
                rot = w * dt;
                trans = v * dt;
                
                % propagate the particle!
                obj.p.x(1,i) = obj.p.x(1,i) + trans * ...
                    cos(obj.p.x(3,i));
                obj.p.x(2,i) = obj.p.x(2,i) + trans * ...
                    sin(obj.p.x(3,i));
                obj.p.x(3,i) = wrapToPi(obj.p.x(3,i) + rot); 
            end
        end
        
        function measurement_update(obj, robot_num, z, useTrustFactor, trustFactorTime)
            % z format - list of observations [range ; bearing ; id]
            % num observations - length of first row
            num_observations = size(z(1,:),2);
            % If we only have 1 obs, then we can't accurately update w meas
            if(num_observations < 2)
%                 disp("Not enough observations, skipping update");
                return
            end
            
            global FIELDINFO;
            global ROBOT_ESTIMATES;
            global MEAS_STATS;
            
            
            % x,y
            lm = zeros(num_observations,2);
            
            for j = 1:num_observations
                if(z(3,j) > 5)
                    % Likely landmark check
                    % The measurements may have the wrong landmark! Ignore
                    % the observation if the perceived landmark range and
                    % bearing is more than landmarkDistanceThreshold 
                    
                    x_t = ROBOT_ESTIMATES(robot_num,1);
                    y_t = ROBOT_ESTIMATES(robot_num,2);
                    theta_t = ROBOT_ESTIMATES(robot_num,3);
                    
                    x1 = x_t + z(1,j)*cos(z(2,j) + theta_t);
                    y1 = y_t + z(1,j)*sin(z(2,j) + theta_t);
                    landmarkX = FIELDINFO(z(3,j) - 5, 2);
                    landmarkY = FIELDINFO(z(3,j) - 5, 3);
                    
                    distanceSquared = (landmarkX-x1)^2+(landmarkY-y1)^2;
                    if distanceSquared > 0.5
%                         eval(['countBadObservationIDMismatch' num2str(id) ...
%                             ' = countBadObservationIDMismatch' num2str(id) ' + 1;'])
%                         % see if there are any other likely landmarks, 
                        foundLikely = false;
                        bestLikelyDistance = Inf;
                        bestLikelyLandmarkId = 0;
                        bestLikelyLandmarkX = 0;
                        bestLikelyLandmarkY = 0;
                        for findLikelyLandmark=1:15
                            currLandmarkId = FIELDINFO(findLikelyLandmark,1);
                            currLandmarkX = FIELDINFO(findLikelyLandmark,2);
                            currLandmarkY = FIELDINFO(findLikelyLandmark,3);
                            
                            currDistanceSquared = (currLandmarkX-x1)^2+(currLandmarkY-y1)^2;
                            if currDistanceSquared < bestLikelyDistance && currDistanceSquared < 0.25
                                foundLikely = true;
                                bestLikelyDistance = currDistanceSquared;
                                bestLikelyLandmarkId = currLandmarkId;
                                bestLikelyLandmarkX = currLandmarkX;
                                bestLikelyLandmarkY = currLandmarkY;
                            end
                        end
                        
                        if foundLikely == true
%                             disp(['Found likely landmark to be ' num2str(bestLikelyLandmarkId) 'instead of ' num2str(idObserved)]);
                            lm(j,1) = bestLikelyLandmarkX;
                            lm(j,2) = bestLikelyLandmarkY;
                            %idObserved = bestLikelyLandmarkId;
                            %likelyLandmarbarcodeId = Barcodes(find(Barcodes(:,1)==bestLikelyLandmarkId),2);
                            %measurements(j,1) = likelyLandmarbarcodeId;
%                             eval(['fixedBadObservationIDMismatch' num2str(id) ...
%                                 ' = fixedBadObservationIDMismatch' num2str(id) ' + 1;'])
                        else
                            continue;
                        end
                    else
                        % If static landmark, just pull from field info
                        lm(j,1) = FIELDINFO(z(3,j) - 5, 2);
                        lm(j,2) = FIELDINFO(z(3,j) - 5, 3);
                    end
                else
                    % If robot, pull from current estimate
                    % If more than 20 sec since last, don't trust
                    if(useTrustFactor && ROBOT_ESTIMATES(z(3,j),4) > trustFactorTime * 50)
                        % Skip update
                        return;
                    else
                        lm(j,1) = ROBOT_ESTIMATES(z(3,j), 1);
                        lm(j,2) = ROBOT_ESTIMATES(z(3,j), 2);
                    end
                end
            end
            
            % Measure how many >2 measurements we get
            MEAS_STATS(robot_num, 3) = MEAS_STATS(robot_num, 3) + 1;
             
            w = ones(obj.n,1); % importance weights
            
            for j = 1:num_observations
                w_prev = w;
                if(lm(j,1) == -1)
                    continue;
                end
                for i = 1:obj.n
                    x_t = obj.p.x(1,i);
                    y_t = obj.p.x(2,i);
                    theta_t = obj.p.x(3,i);
                
                    if(z(1,j) < 0.1)
                        continue;
                    end
                    q = (lm(j,1) - x_t)^2 + (lm(j,2) - y_t)^2;
                    range_e = sqrt(q);
                    bearing_e = wrapToPi(atan2(lm(j,2) - y_t, lm(j,1) ...
                        - x_t) - theta_t);

                    range_error = z(1,j) - range_e;
                    % Skip estimate if bad reading
                    if range_error > 1.0 || abs(bearing_e) > pi/2
                        w = w_prev;
                        break;
                    end
                    bearing_error = wrapToPi(z(2,j) - bearing_e);
                    prob_range = mvnpdf(range_error, 0, 0.15);
                    prob_bearing = mvnpdf(bearing_error, 0, 0.15);
                    w(i) = w(i) * (prob_range * prob_bearing);
                end
            end
            % update and normalize weights
            obj.p.w = w;
            obj.p.w = obj.p.w ./ sum(obj.p.w);

            % If we lose localization, spread weights and cross fingers
            if(sum(obj.p.w) == 0 || isnan(obj.p.w(1)))
                obj.p.w = ones(obj.n,1);
                obj.p.w = obj.p.w ./ sum(obj.p.w);
            end
            
            s = RandStream('mlfg6331_64');
            new_idx = datasample(s, 1:obj.n, obj.n, 'Replace',true,'Weights',obj.p.w);

            for i = 1:obj.n
                obj.p.x(:,i) = obj.p.x(:,new_idx(i));
            end
            
            % Set time since last update to 0
            ROBOT_ESTIMATES(robot_num, 4) = 0;

        end 
        
        function resampling(obj)
            % Importance Resampling
            % Caused estimate to drift wildly
            s = RandStream('mlfg6331_64');
            new_idx = datasample(s, 1:obj.n, obj.n, 'Replace',true,'Weights',obj.p.w);
            
            for i = 1:obj.n
                obj.p.x(:,i) = obj.p.x(:,new_idx(i));
            end
        end
        
        function meanAndVariance(obj)
            obj.mu = mean(obj.p.x, 2); 
            % orientation is a bit more tricky.
            sinSum = 0;
            cosSum = 0;
            for s = 1:obj.n
                cosSum = cosSum + cos(obj.p.x(3,s));
                sinSum = sinSum + sin(obj.p.x(3,s));
            end
            obj.mu(3) = atan2(sinSum, cosSum);     
            % Compute covariance.
            zeroMean = obj.p.x - repmat(obj.mu, 1, obj.n);
            for s = 1:obj.n
                zeroMean(3,s) = wrapTo2Pi(zeroMean(3,s));
            end
            
            obj.Sigma = zeroMean * zeroMean' / obj.n;
        end
    end
end