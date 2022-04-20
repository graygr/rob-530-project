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
        trust;
        time;
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
            obj.trust = true;
            obj.time = 0;
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
        function correction(obj, z, useGTOnly, useLandmarkOnly, useTrustFactor)
            global ROBOT1 ROBOT2 ROBOT3 ROBOT4 ROBOT5 LANDMARK BAR;
            bar = BAR;
            success = false;
            for k = 1:size(z, 1)
                if useGTOnly == true
                    step = find(abs(ROBOT1(:, 1)-z(k, 1))<0.0001);
                    idx = find(bar(:, 2) == z(k, 2));
                    if isempty(idx)
                        continue
                    end
                    switch idx
                        case 1
                            landmark = ROBOT1(step, 2:3);
                        case 2
                            landmark = ROBOT2(step, 2:3);
                        case 3

                            landmark = ROBOT3(step, 2:3);
                        case 4

                            landmark = ROBOT4(step, 2:3);
                        case 5
                            landmark = ROBOT5(step, 2:3);
                        otherwise
                            landmark = LANDMARK(idx-5, 2:3)+LANDMARK(idx-5, 4:5).*randn(1, 2);
                    end
                elseif useLandmarkOnly == true
                    idx = find(bar(:, 2) == z(k, 2));
                    if isempty(idx)
                        continue
                    end
                    switch idx
                        case 1
                            continue
                        case 2
                            continue
                        case 3
                            continue
                        case 4
                            continue
                        case 5
                            continue
                        otherwise
                            landmark = LANDMARK(idx-5, 2:3)+LANDMARK(idx-5, 4:5).*randn(1, 2);
                    end
                elseif useTrustFactor == true
                    idx = find(bar(:, 2) == z(k, 2));
                    if isempty(idx)
                        continue
                    end
                    switch idx
                        case 1
                            if ROBOT1.trust == true
                                landmark = ROBOT1.mu+(ROBOT1.Sigma*randn(3, 1))';
                            else
                                continue
                            end
                        case 2
                             if ROBOT2.trust == true
                                landmark = ROBOT2.mu+(ROBOT2.Sigma*randn(3, 1))';
                            else
                                continue
                            end
                        case 3
                            if ROBOT3.trust == true
                                landmark = ROBOT3.mu+(ROBOT3.Sigma*randn(3, 1))';
                            else
                                continue
                            end
                        case 4
                            if ROBOT4.trust == true
                                landmark = ROBOT4.mu+(ROBOT4.Sigma*randn(3, 1))';
                            else
                                continue
                            end
                        case 5
                            if ROBOT5.trust == true
                                landmark = ROBOT5.mu+(ROBOT5.Sigma*randn(3, 1))';
                            else
                                continue
                            end
                        otherwise
                            landmark = LANDMARK(idx-5, 2:3)+LANDMARK(idx-5, 4:5).*randn(1, 2);
                    end
                else
                    idx = find(bar(:, 2) == z(k, 2));
                    if isempty(idx)
                        continue
                    end
                    switch idx
                        case 1
                            landmark = ROBOT1.mu+(ROBOT1.Sigma*randn(3, 1))';
                        case 2
                            landmark = ROBOT2.mu+(ROBOT2.Sigma*randn(3, 1))';
                        case 3
                            landmark = ROBOT3.mu+(ROBOT3.Sigma*randn(3, 1))';
                        case 4
                            landmark = ROBOT4.mu+(ROBOT4.Sigma*randn(3, 1))';
                        case 5
                            landmark = ROBOT5.mu+(ROBOT5.Sigma*randn(3, 1))';
                        otherwise
                            landmark = LANDMARK(idx-5, 2:3)+LANDMARK(idx-5, 4:5).*randn(1, 2);
                    end
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
                if abs(v(2))>pi/3
                    continue
                elseif abs(v(1))>1.2 && idx<5
                    continue
                elseif abs(v(1))>1.2 && idx>5
                    foundlikely = false;
                    bestdistance = Inf;
                    bestid = 0;
                    for findid = 1:15
                        currx = LANDMARK(findid, 2);
                        curry = LANDMARK(findid, 3);
                        currdistance = (currx-obj.mu_pred(1))^2 + (curry-obj.mu_pred(2))^2;
                        if currdistance<bestdistance && currdistance<0.5
                            foundlikely = true;
                            bestdistance = currdistance;
                            bestid = findid;
                        end
                    end
                    if foundlikely == false
                        continue
                    else
                        landmark = LANDMARK(bestid, 2:3)+LANDMARK(bestid, 4:5).*randn(1, 2);
                    end
                end
                z_hat = 0;
                obj.Y = [];
                for i = 1:2*obj.n+1
                    obj.Y(:, i) = obj.hfun(landmark(1), landmark(2), obj.X(:,i));
                    z_hat = z_hat + obj.w(i) * obj.Y(:,i);
                end
                v = z(k, 3:4)' - z_hat;
                v(2) = wrapToPi(v(2));
                S =  (obj.Y-z_hat) * diag(obj.w) * (obj.Y-z_hat)' + obj.Q;
                Cov_xz = (obj.X-obj.mu_pred) * diag(obj.w) * (obj.Y-z_hat)';
                K = Cov_xz * (S \ eye(size(S)));
                obj.mu_pred = obj.mu_pred + K * v;
                obj.Sigma_pred = obj.Sigma_pred - K * S * K';
                success = true;
            end
            obj.mu = obj.mu_pred;
            obj.Sigma = obj.Sigma_pred;
            if success == true
                obj.trust = true;
                obj.time = 0;
            end
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