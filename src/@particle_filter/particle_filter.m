classdef particle_filter < handle
    % Particle filter class for state estimation of a nonlinear system
    % The implementation follows the Sample Importance Resampling (SIR)
    % filter a.k.a bootstrap filter.
    %
    %   Author: Maani Ghaffari Jadidi
    %   Date:   01/22/2019
    
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
%         dt;             % sampling time
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
            
%             dt = t - obj.last_timestep;
            dt = 0.02;
            
            % For each particle
            for i = 1:obj.n
                % sample noise
                % Noise is set to max(Odo) / 2
                % Max trans is 0.02
                % CHANGE: set noise relative to command 
                %v = normrnd(control(1), 0.005); % trans
                % Max rot is 0.57
                %w = normrnd(control(2), 0.05); % rot
                v = normrnd(control(1), 0.05 + abs(0.3*control(1))); % trans
                % Max rot is 0.57
                w = normrnd(control(2), 0.05 + abs(0.5*control(2))); % rot
                
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
        
        function measurement_update(obj, robot_num, z)
            % z format - list of observations [range ; bearing ; id]
            % num observations - length of first row
            num_observations = size(z(1,:),2);
%             disp(num_observations);
            % If we only have 1 obs, then we can't accurately update w meas
%             if(num_observations < 2)
%                 disp("Not enough observations, skipping update");
%                 return
%             end
            
            global FIELDINFO;
            global ROBOT_ESTIMATES;
            global MEAS_STATS;
            % Measure how many >2 measurements we get
            MEAS_STATS(robot_num, 3) = MEAS_STATS(robot_num, 3) + 1;
            
            % x,y
            lm = zeros(num_observations,2);
            
            for j = 1:num_observations
                if(z(3,j) > 5)
                    % If static landmark, just pull from field info
                    lm(j,1) = FIELDINFO(z(3,j) - 5, 2);
                    lm(j,2) = FIELDINFO(z(3,j) - 5, 3);
                else
                    % If robot, pull from current estimate
                    lm(j,1) = ROBOT_ESTIMATES(z(3,j), 1);
                    lm(j,2) = ROBOT_ESTIMATES(z(3,j), 2);
                end
            end
             
            w = ones(obj.n,1); % importance weights
            for i = 1:obj.n
                x_t = obj.p.x(1,i);
                y_t = obj.p.x(2,i);
                theta_t = obj.p.x(3,i);
                
                for j = 1:num_observations
                    % TODO: Tried limiting observations, see what happens
%                     if(z(1,j) < 0.1)
%                         continue;
%                     end
                    q = (lm(j,1) - x_t)^2 + (lm(j,2) - y_t)^2;
                    range_e = sqrt(q);
                    bearing_e = wrapToPi(atan2(lm(j,2) - y_t, lm(j,1) ...
                        - x_t) - theta_t);

                    range_error = z(1,j) - range_e;
                    bearing_error = wrapToPi(z(2,j) - bearing_e);
                    prob_range = mvnpdf(range_error, 0, 0.08);
                    prob_bearing = mvnpdf(bearing_error, 0, 0.08);
                    w(i) = w(i) * (prob_range * prob_bearing)^2;
                end
            end
            % update and normalize weights
            obj.p.w = w;
            obj.p.w = obj.p.w ./ sum(obj.p.w);

            % Importance Resampling
%             disp(w)

            % If we lose localization, spread weights and cross fingers
            if(sum(obj.p.w) == 0)
                obj.p.w = ones(obj.n,1);
                obj.p.w = obj.p.w ./ sum(obj.p.w);
            end
            
%             disp(obj.p.w);
            
            s = RandStream('mlfg6331_64');
            new_idx = datasample(s, 1:obj.n, obj.n, 'Replace',true,'Weights',obj.p.w);

            for i = 1:obj.n
                obj.p.x(:,i) = obj.p.x(:,new_idx(i));
            end

            % compute effective number of particles
%             obj.Neff = 1 / sum(obj.p.w.^2);
        end 
        
        function resampling(obj)
%             % low variance resampling
%             W = cumsum(obj.p.w);          
%             r = rand / obj.n ;
%             j = 1;
%             for i = 1:obj.n
%                 u = r + (i-1) / obj.n;
%                 while u > W(j)
%                     j = j + 1;
%                 end
%                 obj.p.x(:,i) = obj.p.x(:,j);
%                 obj.p.w(i) = 1/obj.n;
%             end
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