classdef EKF < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        gfun;           % Discrete Dynamical equations of motion
        hfun;           % Measurement model equations
        Gfun;           % Motion Model Jacobian with respect to state
        Vfun;           % Motion Model Jacobian with respect to control inputs
        Hfun;           % Measruement Model Jacobian
        qfun;           % noise covariance function respective to motion commands
        M;              % Noise Covariance in input measruements(This is dynamical, changes with the input values)
        Q;              % Sensor Noise Covariance
        q;              % sensor noise covariance from qfun
        mu_pred;        % Predictied Mean after prediction step
        Sigma_pred;     % Predictied Covarince after prediction step
    end
    
    methods
        function obj = EKF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % Jocabian of motion model
            obj.Gfun = init.Gfun;
            obj.Vfun = init.Vfun;
            % Jocabian of measurement model
            obj.Hfun = init.Hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            obj.qfun = sys.qfun;
            % initial mean and covariance
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        %% Complete prediction function
        function prediction(obj, u)
            obj.mu_pred = obj.gfun(obj.mu, u); %Predicted mean
            obj.Sigma_pred = obj.Gfun(obj.mu, u) * obj.Sigma *obj.Gfun(obj.mu, u)' + ...
                obj.Vfun(obj.mu, u) * obj.M(u) * obj.Vfun(obj.mu, u)'; %predicted covariance
            obj.q = obj.qfun(u);
        end
        
        function setPredictionAsCurrent(obj)
            obj.mu = obj.mu_pred;
            obj.mu(3) = wrapToPi(obj.mu(3));
            obj.Sigma = obj.Sigma_pred; 
        end
        
        %% Complete correction function
        function correction(obj, z)
            % assumes there are at least two observations 
            % observations will be [bearing1, bearing2, ...
            %                       range1,   range2, ...
            %                       xglobalpos1, xglobalpos2,...
            %                       yglobalpos1, yglobalpos2,... ]
            numObservations = size(z,2);
            if (numObservations < 2)
                msg=['There must be at least 2 observations and there are only:',num2str(numObservations)];
                error(msg);
            end
            zExtracted = []; % Extract the z from the input and stack them accordingly
            z_hats = [];
            H = [];
            Qblk = [];
            for i=1:numObservations
                zExtracted = [zExtracted; wrapToPi(z(1,i)); z(2,i)];
                xObs = z(3,i);
                yObs = z(4,i);
                z_hati = obj.hfun(xObs, yObs, obj.mu_pred);
                z_hati(1) = wrapToPi(z_hati(1));
                z_hats = [z_hats; z_hati];
                Hi = obj.Hfun(xObs,yObs, obj.mu_pred, z_hati);
                H = [H; Hi];
                Qblk = blkdiag(Qblk, obj.Q);
%                 Qblk = blkdiag(Qblk, obj.q);
            end
    
            S = H* obj.Sigma_pred * H' + Qblk;
            K = obj.Sigma_pred * H' * (S \ eye(size(S)));
            
            
            diff = zExtracted - z_hats;
            %go through and wraptopi all the bearing diffs
            for i=1:numObservations
                index = 2*i-1;
                diff(index) = wrapToPi(diff(index));
            end
            
            obj.mu = obj.mu_pred + K *diff;
            obj.mu(3) = wrapToPi(obj.mu(3));
            I = eye(length(obj.mu));
            obj.Sigma = (I - K * H) * obj.Sigma_pred * (I - K * H)' ...
                    + K * Qblk * K'; % Joseph update form
        end
    end
end

