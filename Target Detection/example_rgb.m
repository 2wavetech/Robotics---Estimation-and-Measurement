% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = './train';
Samples = [];
for k=1:15
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);       % the "red" values of all the pixels
    G = I(:,:,2);       % the "green" values of all the pixels
    B = I(:,:,3);       % the "blue" values of all the pixels
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);        % the ball "filter", i.e., the locations of the while pixels counted in serial way, not (row, col).
    
    R = R(sample_ind);                  % the "red" components of the ball, sample_ind provides locations of pixels of the ball in the picture
    G = G(sample_ind);                  % the "green" components of the ball
    B = B(sample_ind);                  % the "blue" components of the ball
    
    Samples = [Samples; [R G B]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end

% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');    % to dsiplay the distribution over "red", "green" and "blue" axis
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iterations = 50;
K = 2;
[mu1, sigma1] = GMM(R, G, B, K, 1);
[mu, sigma] = GMM(R, G, B, K, iterations);
for i = 1:K
    mu{i} = mu1{i} - mu{i};
    sigma{i} = sigma1{i} - sigma{i};
end 
