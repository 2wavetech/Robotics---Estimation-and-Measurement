%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function computes the multivariate Gaussian of the input data x
% given the mean vector mu and covariance matrix sigma. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function G = multivariateGaussian(x, mu, sigma)
    D = size(x, 2);                 % the dimensions of each sample
    Y = double(x) - mu;             % MxD
    %**********************************************************************
    % X = inv(A)*B is theoretically the same as X = A\B and Y = B*inv(A) is
    % theoretically the same as Y = B/A.
    %**********************************************************************
    H = sum(Y/sigma .* Y, 2);             % "/" avoids using inverse of sigma.
    C = log(det(sigma)) + D * log(2*pi());
    G = exp((H + C)/(-2));
end