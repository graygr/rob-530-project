 classdef UKF < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        gfun;           % Motion Model Function
        hfun;           % Measruement Model Function
        M;              % Motion model noise(dynamical and function of input)
        Q;              % Sensor Noise
        kappa_g;        
        mu_pred;
        Sigma_pred;
        n;              % Number of Sigma points
        X;              % Sigma points
        w;              % Weight of Sigma points
        Y;              % Sigma points after propagation
    end
    
    methods
        function obj = UKF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            obj.kappa_g = init.kappa_g;
            % initial mean and covariance
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        %% Prediction function
        function prediction(obj, u)
            sigma_point(obj, obj.mu, obj.Sigma, obj.kappa_g);
            obj.mu_pred = 0;
            for i = 1:2*obj.n+1
                obj.Y(:,i) = obj.gfun(obj.X(:,i), u);
                obj.mu_pred = obj.mu_pred + obj.w(i) * obj.Y(:,i);
            end
            obj.Sigma_pred = (obj.Y-obj.mu_pred) * diag(obj.w) * (obj.Y-obj.mu_pred)' + obj.M(u);
        end
        
        %% Correction function
        function correction(obj, z)
            global ROBOT1 ROBOT2 ROBOT3 ROBOT4 ROBOT5 LANDMARK BAR;
            bar = BAR;
            for k = 1:size(z, 1)
%                 step = find(abs(ROBOT1(:, 1)-z(k, 1))<0.0001);
                idx = find(bar(:, 2) == z(k, 2));
                if isempty(idx)
                    continue
                end
                switch idx
                    case 1
                        landmark = ROBOT1.mu+(ROBOT1.Sigma*randn(3, 1))';
%                         landmark = ROBOT1(step, 2:3);
                    case 2
                        landmark = ROBOT2.mu+(ROBOT2.Sigma*randn(3, 1))';
%                         landmark = ROBOT2(step, 2:3);
                    case 3
                        landmark = ROBOT3.mu+(ROBOT3.Sigma*randn(3, 1))';
%                         landmark = ROBOT3(step, 2:3);
                    case 4
                        landmark = ROBOT4.mu+(ROBOT4.Sigma*randn(3, 1))';
%                         landmark = ROBOT4(step, 2:3);
                    case 5
                        landmark = ROBOT5.mu+(ROBOT5.Sigma*randn(3, 1))';
%                         landmark = ROBOT5(step, 2:3);
                    otherwise
                        landmark = LANDMARK(idx-5, 2:3)+LANDMARK(idx-5, 4:5).*randn(1, 2);
                end
                sigma_point(obj, obj.mu_pred, obj.Sigma_pred, obj.kappa_g);
                z_hat = 0;
                obj.Y = [];
                for i = 1:2*obj.n+1
                    obj.Y(:, i) = obj.hfun(landmark(1), landmark(2), obj.X(:,i));
                    z_hat = z_hat + obj.w(i) * obj.Y(:,i);
                end
                v = z(k, 3:4)' - z_hat;
                v(2) = wrapToPi(v(2));
                if abs(v(1))>1.2 || abs(v(2))>pi/3
                    continue
                end
                S =  (obj.Y-z_hat) * diag(obj.w) * (obj.Y-z_hat)' + obj.Q;
                Cov_xz = (obj.X-obj.mu_pred) * diag(obj.w) * (obj.Y-z_hat)';
                K = Cov_xz * (S \ eye(size(S)));
                obj.mu_pred = obj.mu_pred + K * v;
                obj.Sigma_pred = obj.Sigma_pred - K * S * K';
            end
            obj.mu = obj.mu_pred;
            obj.Sigma = obj.Sigma_pred;
        end
        
        function sigma_point(obj, mean, cov, kappa)
            obj.n = numel(mean);
            L = sqrt(obj.n + kappa) * chol(cov,'lower');
            obj.Y = mean(:,ones(1, numel(mean)));
            obj.X = [mean,obj.Y + L, obj.Y - L];
            obj.w = zeros(2 * obj.n + 1,1);
            obj.w(1) = kappa / (obj.n + kappa);
            obj.w(2:end) = 0.5 / (obj.n + kappa);
        end
    end
end