function G = multivariateGaussian(x, mu, sigma)
    Dimension = size(x, 2);                 % the dimensions of each sample
    Y = double(x) - mu;                     % 1xD
    Hi = (Y/sigma * Y')*(-0.5);             % "/" avoids using inverse of sigma.
    G = (1/((2 * pi())^(Dimension/2))*(det(sigma))^0.5) * exp(Hi);
end