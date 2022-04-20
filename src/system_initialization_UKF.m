function sys = system_initialization_UKF(alphas, beta, gamma, dt)
sys.gfun = @(mu, u) [...
    mu(1) + u(1)*dt*cos(mu(3)+u(2)*dt);
    mu(2) + u(1)*dt*sin(mu(3)+u(2)*dt);
    mu(3) + u(2)*dt];

sys.hfun = @(landmark_x, landmark_y, mu_pred) [...
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2);
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3))];

sys.M = @(u)[alphas(1)*u(1)^2+alphas(2)*u(2)^2, 0, 0; 
                       0, alphas(3)*u(1)^2+alphas(4)*u(2)^2, 0;
                       0, 0, alphas(5)*u(1)^2+alphas(6)*u(2)^2];

sys.Q = [...
        beta^2,    0;
        0,      gamma^2];
end