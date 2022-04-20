%-------------------------------------------------------
% noisy version of motion control
% motion in form of [drot1,dtrans,drot2]
%-------------------------------------------------------
function noisymotion = sampleOdometry(motion, alphas)

Trans_vel = motion(1);
Angular_vel = motion(2);

noisymotion(1)=mvnrnd(motion(1),alphas(1)*Trans_vel^2+alphas(2)*Angular_vel^2);
noisymotion(2)=mvnrnd(motion(2),alphas(3)*Trans_vel^2+alphas(4)*Angular_vel^2);
noisymotion(3)=mvnrnd(motion(3),alphas(5)*Trans_vel^2+alphas(6)*Angular_vel^2); 

