% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is the script code for collecting ball sample colors using roipoly
% and calling GMM function to learn the Guassian model for the ball image

close all

imagepath = './train';
Samples = [];

I = imread(sprintf('%s/ball.png',imagepath));
    
% You may consider other color space than RGB
R = I(:,:,1);       % the "red" values of all the pixels
G = I(:,:,2);       % the "green" values of all the pixels
B = I(:,:,3);       % the "blue" values of all the pixels

% get the mask of the ball in the picture
figure(1),          
mask = roipoly(I); 

% display teh mask (optional, with "1" on the ball and "0" on the rest of the picture
figure(2), imshow(mask); title('Mask');       

% the ball "filter", i.e., the locations of the while pixels counted in serial way, not in (row, col)
sample_ind = find(mask > 0);       
R = R(sample_ind);                  % the "red" components of the ball, sample_ind provides locations of pixels of the ball in the picture
G = G(sample_ind);                  % the "green" components of the ball
B = B(sample_ind);                  % the "blue" components of the ball
    
Samples = [Samples; [R G B]];
 
% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');    % to dsiplay the distribution over "red", "green" and "blue" axis
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% call GMM to learn the Gaussiam models of the ball image                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
iterations = 2000;    % the number of iterations of GMM
K = 1;              % the number of Gaussian modes
% [mu1, sigma1] = GMM(R, G, B, K, 1);
[mu, sigma] = GMM(R, G, B, K, iterations);
for i = 1:K
    mu1{i} = mu1{i} - mu{i};
    sigma1{i} = sigma1{i} - sigma{i};
end 