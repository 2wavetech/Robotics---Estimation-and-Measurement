function [mu, sigma] =  GMM (R, G, B, K, iterations)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GMM is used to calculate the "converged" mean mu and covariance sigma %
% for the input pixels [R G B], or data points in general.              %
% Inputs:                                                               %
%     X: the matrix of points, e.g.,[R G B], with values on each row    %
%        for one point and on column the dimensions of that point.      %
%     K: the number of multivariate Guassian modes, same for all        %
%        dimensions.                                                    %
%     iterations: the number of iterations                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

X = [R G B];        % the samples, N x D
N = size(X, 1);     % the number of samples
D = size(X, 2);     % the dimensions of each sample

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization of mu and Sigma                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
muVector = [sum(R)/length(R), sum(G)/length(G), sum(B)/length(B)];      % the initial mean vector, assuming single Gaussian mode
variance = rand(N, D);                                                  % allocate space for variance matrix (N x D)

for i = 1:D
    variance(:,i) = double(X(:, i)) - muVector(i);          % calculate variance, NxD
end
Sigma = (variance' * variance) / N;                         % the covariance

muVector = repmat({muVector}, K, 1);                        % build the initial mean vector for K modes, each accessed by muVector{k}
Sigma = repmat({Sigma}, K, 1);                              % build the initial covariance matrix for K modes, each accessed by Sigma{i}

for k = 1:K                                                 % add randomness to the initial mu and Sigma
    muVector{k} = muVector{k} + 50 * rand(1)*k;
    Sigma{k} = Sigma{k} + 50*rand(1)*k;
end

muLast = muVector;                                          % used to measure convergence of GMM iterations
SigmaLast = Sigma;

while iterations >0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate latent variable z(i, k) while fixing mu and sigma           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = double (zeros(1, K));           % Probability of each data point abide by Gaussian distribution
z = double (zeros(N, K));
for i = 1:N                                                 
    for k = 1:K
        g(k) = multivariateGaussian(X(i,:), muVector{k}, Sigma{k});
    end
    sigmaG = sum(g);

    for k = 1:K
        z(i, k) = g(k)/sigmaG;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update mu and Sigma using z(i, k)                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 1:K
    Y = (z(:,k) .* double(X)) / sum(z(:, k)); 
    muVector{k} = sum(Y, 1);
    sumSigma = z(:, k)' .* (double(X) - muVector{k})' * (double(X) - muVector{k});
    Sigma{k} = sumSigma/sum(z(:, k));
end    

iterations = iterations -1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measure the convergence                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k = 1:K
    if ~isempty(find(abs(muLast{k} - muVector{k})>0.01, 1)) || ~isempty(find(abs(SigmaLast{k} - Sigma{k})>0.01, 1))
        muLast{k} = muVector{k};
        SigmaLast{k} = Sigma{k};
    else
        fprintf('The iteration stops at %d\n', iterations);
        iterations = 0;
        break;
    end
end
end
    mu = muLast;                         % Return the final results
    sigma = SigmaLast;
end
