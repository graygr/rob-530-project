function results = mahalanobis(filterMu,filterSigma,GTMu)
    results= zeros(8,1);
    
    diff = filterMu-GTMu;
    diff(3)= wrapToPi(diff(3));
    results(1) = sqrt(diff'*inv(filterSigma)*diff);
    results(2:4) = diff;
    results(5) = 3*sqrt(filterSigma(1,1));
    results(6) = 3*sqrt(filterSigma(2,2));
    results(7) = 3*sqrt(filterSigma(3,3));
    results(8) =sqrt(diff(1)^2+diff(2)^2);
    
% Output format (7D vector) : 1. Mahalanobis distance
%                             2-4. difference between groundtruth and filter estimation in x,y,theta 
%                             5-7. 3*sigma (square root of variance) value of x,y, theta 
%                             8 Euclidean distance

end